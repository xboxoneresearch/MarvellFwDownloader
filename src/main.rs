#![allow(dead_code)]

use std::{io::{Cursor, Read, Seek, SeekFrom, Write}, time::Duration};
use binrw::{
    binrw,
    BinRead,
    BinWrite,
};
use std::thread::sleep;

const DRIVER_USB_BULK_MSG_TIMEOUT: Duration = Duration::from_millis(100);

const MARVELL_USB_FW_DNLD: u8 = 1;
/** Boot state: FW ready */
const MARVELL_USB_FW_READY: u8 = 2;

/** CMD id for CMD7 */
const FW_CMD_7: u32 = 0x00000007;

/** High watermark for Tx data */
const MVUSB_TX_HIGH_WMARK: u8 = 6;

/** Number of Rx data URB */
const MVUSB_RX_DATA_URB: u8 = 6;

/* Transmit buffer size for chip revision check */
const CHIP_REV_TX_BUF_SIZE: usize = 16;
/* Receive buffer size for chip revision check */
const CHIP_REV_RX_BUF_SIZE: usize = 2048;

/* Extensions */
const EXTEND_HDR: u32 = 0xAB95;
const EXTEND_V1: u32 = 0x0001;

/** USB8797 chip revision ID */
const USB8797_A0: u32 = 0x00000000;
const USB8797_B0: u32 = 0x03800010;

/** Tx buffer size for firmware download*/
const FW_DNLD_TX_BUF_SIZE: usize = 620;
/** Rx buffer size for firmware download*/
const FW_DNLD_RX_BUF_SIZE: usize = 2048;
/** Max firmware retry */
const MAX_FW_RETRY: u8 = 3;

/** Firmware has last block */
const FW_HAS_LAST_BLOCK: u32 = 0x00000004;

/** Firmware data transmit size */
fn fw_data_xmit_size(data_len: u32) -> u32 {
    std::mem::size_of::<FWHeader>() as u32 + data_len + std::mem::size_of::<u32>() as u32
}

/** FWHeader */
#[binrw]
#[brw(little)]
#[derive(Debug, Clone)]
struct FWHeader {
    /** FW download command */
	dnld_cmd: u32,
    /** FW base address */
	base_addr: u32,
    /** FW data length */
	data_length: u32,
    /** FW CRC */
	crc: u32,
}

/** FWData */
#[binrw]
#[brw(little)]
#[derive(Debug)]
struct FWData {
    /** FW data header */
	fw_header: FWHeader,
    /** FW data sequence number */
	seq_num: u32,
    /* FW data buffer */
	// data: [u8; 2],
}

/** FWSyncHeader */
#[binrw]
#[brw(little)]
#[derive(Debug)]
struct FWSyncHeader {
    /** FW sync header command */
	cmd: u32,
    /** FW sync header sequence number */
	seq_num: u32,
}

#[binrw]
#[brw(little)]
#[derive(Debug)]
struct UsbAckPkt {
    ack_winner: u32,
    seq: u32,
    extend: u32,
    chip_rev: u32,
}

#[repr(u8)]
#[derive(Debug)]
enum DriveUsbEp {
    Ctrl = 0,
    CmdEvent = 1,
    Data = 2,
}

#[warn(non_camel_case_types)]
#[derive(Debug)]
enum MarvellChip {
    Avastar88W8782U,
    Avastar88W8897
}

fn read_fw(path: &str) -> Result<Vec<u8>, std::io::Error> {
    let mut file = std::fs::File::open(path)?;
    file.seek(SeekFrom::End(0))?;
    let fsize = file.stream_position()?;
    file.seek(SeekFrom::Start(0))?;

    let mut buf = vec![0u8; fsize as usize];
    file.read_exact(&mut buf)?;

    Ok(buf)
}

fn program_fw<T: rusb::UsbContext>(handle: &rusb::DeviceHandle<T>, fw: &[u8]) -> Result<(), Box<dyn std::error::Error>> {
    let mut data_len = 0;
    let mut seq_num = 0;
    let mut reader = Cursor::new(fw);

    let mut retries = MAX_FW_RETRY;

    while retries > 0 {
        let fw_header = FWHeader::read(&mut reader)?;
        println!("[*] FW Header: {fw_header:?}");
        data_len = fw_header.data_length;

        /* CMD 7 don't have data_length filed */
        if fw_header.dnld_cmd == FW_CMD_7 {
            data_len = 0;
        }
        let mut data_buf = vec![0u8; data_len as usize];
        reader.read_exact(&mut data_buf)?;

        // Prepare fw block to send
        let fw_data = FWData {
            fw_header: fw_header.clone(),
            seq_num
        };
        
        while retries > 0 {
            // Send block
            println!("[*] Sending packet, seq: {seq_num}");
            let mut send_buffer = vec![];
            let mut writer = Cursor::new(&mut send_buffer);
            // Write fw header + sequence
            fw_data.write(&mut writer)?;

            // Append data portion
            writer.write(&data_buf)?;
            if let Err(_) = handle.write_bulk(0x01, &send_buffer, DRIVER_USB_BULK_MSG_TIMEOUT) {
                println!("[-] Failed when sending packet...");
                retries -= 1;
                sleep(Duration::from_millis(100));
                continue;
            }

            // Receive sync response
            let mut recv_buffer = vec![0u8; FW_DNLD_RX_BUF_SIZE];
            
            if let Err(_) = handle.read_bulk(0x81, &mut recv_buffer, DRIVER_USB_BULK_MSG_TIMEOUT) {
                println!("[-] Failed when receiving packet...");
                retries -= 1;
                sleep(Duration::from_millis(100));
                continue;
            }

            let sync_header = FWSyncHeader::read(&mut Cursor::new(&recv_buffer))?;
            println!("[*] Sync header: {sync_header:?}");
        
            if sync_header.cmd > 0 {
                return Err("FW received block with CRC error".into());
            }
            else if sync_header.seq_num != seq_num {
                return Err(format!("Mismatch in seq, got {}, expected: {seq_num}", sync_header.seq_num).into());
            }
            else if fw_header.dnld_cmd == FW_HAS_LAST_BLOCK {
                println!("[+] Last block - finished!");
                return Ok(());
            }

            // Block transmitted successfully, reset retry count
            retries = MAX_FW_RETRY;
            break;
        }
        seq_num += 1;
    }

    return Err("Fw download did not succeed".into());
}

fn check_chip_rev<T: rusb::UsbContext>(handle: &rusb::DeviceHandle<T>) -> Result<(), Box<dyn std::error::Error>> {
    let extend = (EXTEND_HDR << 16) | EXTEND_V1;
    let send_buf = vec![0u8; CHIP_REV_TX_BUF_SIZE];
    let mut recv_buf = vec![0u8; CHIP_REV_RX_BUF_SIZE];
    handle.write_bulk(0x01, &send_buf, DRIVER_USB_BULK_MSG_TIMEOUT)?;
    handle.read_bulk(0x81, &mut recv_buf, DRIVER_USB_BULK_MSG_TIMEOUT)?;

    let pkt = UsbAckPkt::read(&mut Cursor::new(&mut recv_buf))?;
    println!("[*] Chiprev resp: {pkt:?}");
    
    if pkt.extend == extend {
        println!("[*] Chip Rev: {} (From Response)", pkt.chip_rev);
    } else {
        println!("[*] Chip Rev: {}", USB8797_A0);
    }

    Ok(())
}

fn download_fw<T: rusb::UsbContext>(chip: MarvellChip, device: rusb::Device<T>, fw_path: &str) -> Result<(), Box<dyn std::error::Error>> {
    println!("[+] Starting fw download for {:?}", chip);
    let fw = read_fw(fw_path)?;
    println!("[+] Read fw {fw_path} ({} bytes)", fw.len());

    let mut handle = device.open()?;

    // Ignore error, windows will throw one
    let _ = handle.set_auto_detach_kernel_driver(true);

    handle.claim_interface(0)?;

    check_chip_rev(&handle)?;
    program_fw(&handle, &fw)?;

    handle.release_interface(0)?;

    //handle.reset()?;

    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        return Err(format!("Usage: {} [fw filepath]", &args.first().unwrap()).into());
    }

    let fw_filepath = &args[1];

    for device in rusb::devices().unwrap().iter() {
        let device_desc = device.device_descriptor().unwrap();
        if device_desc.vendor_id() == 0x1286 {
            println!("[*] Found marvell device: Bus {:03} Device {:03} ID {:04x}:{:04x}",
                device.bus_number(),
                device.address(),
                device_desc.vendor_id(),
                device_desc.product_id());

            let chip = match device_desc.product_id() {
                0x2040 => {
                    MarvellChip::Avastar88W8782U
                },
                0x2045 => {
                    MarvellChip::Avastar88W8897
                },
                pid => {
                    return Err(format!("Unhandled marvell device with pid: {:#X}", pid).into());
                }
            };

            println!("[*] {chip:?}");
            download_fw(chip, device, &fw_filepath)?;

            return Ok(());
        }
    }

    return Err("No device found!".into());
}
