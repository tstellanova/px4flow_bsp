/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_std]

///! This crate provides some commonly useful interfaces to the
///! PX4FLOW board, such as:
///! - Camera configuration/control (via the [mt9v034-i2c crate](https://crates.io/crates/mt9v034-i2c))
///! - Six degree of freedom (6DOF) accelerometer and gyroscope sense via the
///!  [l3gd20 crate](https://crates.io/crates/l3gd20)
///! - Read/Write onboard EEPROM
///! - Reading camera image data (via a DCMI `read_available` function)
///! See the example and README for more details.

#[allow(unused)]
pub mod peripherals;

#[allow(unused)]
pub mod board;

#[allow(unused)]
pub mod dcmi;
