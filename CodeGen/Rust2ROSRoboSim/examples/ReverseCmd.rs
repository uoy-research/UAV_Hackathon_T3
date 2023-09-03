#![allow(unused_parens)]
#![allow(unused_imports)]
#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_mut)]
#![allow(non_snake_case)]
#![allow(non_camel_case_types)]

//ROS2 to RUST imports
use futures::executor::LocalPool;
use futures::future;
use futures::stream::StreamExt;
use futures::task::LocalSpawnExt;
use r2r::QosProfile;

use nalgebra::{SVector,SMatrix};
use strum::Display;
use std::thread;
use std::sync::mpsc;
use std::sync::mpsc::{SyncSender, Receiver};
use std::sync::Barrier;
use std::sync::Arc;
use std::time::Duration;
use rand::prelude::*;
// These are only used for the interaction with the user and should be encapsulated in
// whatever extension mechanism that will be developed to allow integration with different
// simulators.
use std::io::Write;
use std::io::Read;
use std::rc::Rc;
use std::cell::RefCell;

use log::debug;
use simplelog::*;
use std::fs::File;

use serde::{Deserialize, Serialize};
use serde_json::Result;
use clap::{App,Arg};

#[derive(PartialEq,Display,Debug,Clone,Serialize,Deserialize)]
enum M_ReverseCmd_input {
	MoveIn(SVector<f32, 6>),
	opticalFlow(f32),
	_done_,
	_terminate_,
}
#[derive(PartialEq,Display,Debug,Clone,Serialize,Deserialize)]
enum M_ReverseCmd_output {
	MoveOut(SVector<f32, 6>),
	Land,
	_done_,
}
#[derive(PartialEq,Display,Debug,Clone,Serialize,Deserialize)]
enum STATUS {
	ENTER_STATE,
	ENTER_CHILDREN,
	EXECUTE_STATE,
	EXIT_CHILDREN,
	EXIT_STATE,
	INACTIVE,
}
#[derive(PartialEq,Display,Debug,Clone,Serialize,Deserialize)]
enum RESULT {
	WAIT,
	CONT,
}
#[derive(PartialEq,Display,Debug,Clone,Serialize,Deserialize)]
enum C_ctrl_ref0_output {
	Land,
	MoveOut(SVector<f32, 6>),
	_done_,
}
#[derive(PartialEq,Display,Debug,Clone,Serialize,Deserialize)]
enum STATES_stm_ref0 {
	NONE,
	SignalReceived,
	ReturnHome,
	Landing,
	final_f0,
	FailureMode,
	Waiting,
}
#[derive(PartialEq,Display,Debug,Clone,Serialize,Deserialize)]
enum stm_ref0_output {
	MoveOut(SVector<f32, 6>),
	Land,
	_done_,
}
#[derive(PartialEq,Display,Debug,Clone,Serialize,Deserialize)]
enum TRANSITIONS_stm_ref0 {
	NONE,
	stm_ref0_t4,
	stm_ref0_t1b,
	stm_ref0_t1a,
	stm_ref0_t6,
	stm_ref0_t7,
	stm_ref0_t2,
	stm_ref0_t3,
	stm_ref0_t8,
	stm_ref0_t1,
	stm_ref0_t5,
	stm_ref0_t0,
}
#[derive(PartialEq,Display,Debug,Clone,Serialize,Deserialize)]
enum C_ctrl_ref0_input {
	MoveIn(SVector<f32, 6>),
	opticalFlow(f32),
	_done_,
	_terminate_,
}
#[derive(PartialEq,Display,Debug,Clone,Serialize,Deserialize)]
enum stm_ref0_input {
	MoveIn(SVector<f32, 6>),
	opticalFlow(f32),
	_done_,
	_terminate_,
}

#[derive(PartialEq,parse_display::Display,Debug,Clone,Serialize,Deserialize)]
#[display("(|{done}, {state}, {target_state}, {status}, {en_ReverseCmdS_ReturnHome_1_done}, {en_ReverseCmdS_ReturnHome_1_counter}, {en_ReverseCmdS_SignalReceived_1_done}, {en_ReverseCmdS_SignalReceived_1_counter}, {tr_ReverseCmdS_t6_done}, {tr_ReverseCmdS_t6_counter}, {tr_ReverseCmdS_t2_done}, {tr_ReverseCmdS_t2_counter}, {tr_ReverseCmdS_t3_done}, {tr_ReverseCmdS_t3_counter}, {tr_ReverseCmdS_t0_done}, {tr_ReverseCmdS_t0_counter}|)")]
struct stm_ref0_state {
	#[display("done -> {:?}")]
	done:bool,
	#[display("state -> {:?}")]
	state:STATES_stm_ref0,
	#[display("target_state -> {:?}")]
	target_state:STATES_stm_ref0,
	#[display("status -> {:?}")]
	status:STATUS,
	#[display("en_ReverseCmdS_ReturnHome_1_done -> {:?}")]
	en_ReverseCmdS_ReturnHome_1_done:bool,
	#[display("en_ReverseCmdS_ReturnHome_1_counter -> {:?}")]
	en_ReverseCmdS_ReturnHome_1_counter:i32,
	#[display("en_ReverseCmdS_SignalReceived_1_done -> {:?}")]
	en_ReverseCmdS_SignalReceived_1_done:bool,
	#[display("en_ReverseCmdS_SignalReceived_1_counter -> {:?}")]
	en_ReverseCmdS_SignalReceived_1_counter:i32,
	#[display("tr_ReverseCmdS_t6_done -> {:?}")]
	tr_ReverseCmdS_t6_done:bool,
	#[display("tr_ReverseCmdS_t6_counter -> {:?}")]
	tr_ReverseCmdS_t6_counter:i32,
	#[display("tr_ReverseCmdS_t2_done -> {:?}")]
	tr_ReverseCmdS_t2_done:bool,
	#[display("tr_ReverseCmdS_t2_counter -> {:?}")]
	tr_ReverseCmdS_t2_counter:i32,
	#[display("tr_ReverseCmdS_t3_done -> {:?}")]
	tr_ReverseCmdS_t3_done:bool,
	#[display("tr_ReverseCmdS_t3_counter -> {:?}")]
	tr_ReverseCmdS_t3_counter:i32,
	#[display("tr_ReverseCmdS_t0_done -> {:?}")]
	tr_ReverseCmdS_t0_done:bool,
	#[display("tr_ReverseCmdS_t0_counter -> {:?}")]
	tr_ReverseCmdS_t0_counter:i32,
}
#[derive(PartialEq,parse_display::Display,Debug,Clone,Serialize,Deserialize)]
#[display("(|{MoveIn}, {MoveIn_value}, {opticalFlow}, {opticalFlow_value}, {_clock_C}, {_clock_T}, {_transition_}|)")]
struct stm_ref0_inputstate {
	#[display("MoveIn -> {:?}")]
	MoveIn:bool,
	#[display("MoveIn_value -> {:?}")]
	MoveIn_value:SVector<f32, 6>,
	#[display("opticalFlow -> {:?}")]
	opticalFlow:bool,
	#[display("opticalFlow_value -> {:?}")]
	opticalFlow_value:f32,
	#[display("_clock_C -> {:?}")]
	_clock_C:i32,
	#[display("_clock_T -> {:?}")]
	_clock_T:i32,
	#[display("_transition_ -> {:?}")]
	_transition_:TRANSITIONS_stm_ref0,
}
#[derive(PartialEq,parse_display::Display,Debug,Clone,Serialize,Deserialize)]
#[display("(|{commands}, {distanceToGround}, {command}, {timeout}, {i}|)")]
struct stm_ref0_memory {
	#[display("commands -> {:?}")]
	commands:Vec<SVector<f32, 6>>,
	#[display("distanceToGround -> {:?}")]
	distanceToGround:f32,
	#[display("command -> {:?}")]
	command:SVector<f32, 6>,
	#[display("timeout -> {:?}")]
	timeout:f32,
	#[display("i -> {:?}")]
	i:i32,
}

fn main() -> std::result::Result<(), String> {
	let matches = App::new("SimCacheCons")
						.version("1.0.0")
						.about("Simulation of RoboSim model SimCacheCons")
						.arg(Arg::with_name("record")
							.short('r')
							.long("record")
							.value_name("FILE")
							.help("Records the state of a simulation in a file.")
							.takes_value(true)
						).arg(Arg::with_name("replay")
							.short('p')
							.long("replay")
							.value_name("FILE")
							.help("Replays the simulation with the contents of a file")
							.takes_value(true)
						).get_matches();
	
	let _ = WriteLogger::init(
		LevelFilter::Trace, Config::default(), File::create("test.log").unwrap());
	
	let _args: Vec<String> = std::env::args().collect();
    if _args.len() <= 0 {
        eprintln!("error: Not enough arguments.");
            std::process::exit(1);
    }

    //r2r code
    let ctx = r2r::Context::create().unwrap();
	let mut node = r2r::Node::create(ctx, "testnode", "").unwrap();//maperr|_|{String::new("string")})?;
    let mut pool = LocalPool::new();
    let spawner = pool.spawner();
    let mut sub = node.subscribe::<r2r::std_msgs::msg::String>("/event", QosProfile::default()).unwrap();
    let publisher = node.create_publisher::<r2r::std_msgs::msg::String>("/operation", QosProfile::default()).unwrap();
    let MoveOutPublisher = node.create_publisher::<r2r::std_msgs::msg::String>("/MoveOut", QosProfile::default()).unwrap();
    let LandPublisher = node.create_publisher::<r2r::std_msgs::msg::String>("/Land", QosProfile::default()).unwrap();

    
    // TODO: This record and replay mechanism needs to be generalised for multiple
    // statemachine and controllers. Currently, only the recording functionality
    // is implemented and only records data for one state machine.
    
    let replay_value = matches.value_of("replay").and_then(|s|Some(s.to_string()));
	let record_value = matches.value_of("record").and_then(|s|Some(s.to_string()));

	let replayFile = if let Some(s) = replay_value {
		let file = std::fs::File::create(s);
		match file {
			Ok(f) => Some(f),
			Err(e) => { eprintln!("{}", e); std::process::exit(1) }
		}
	} else {
		None
	};

	let (send_data, recv_data)= mpsc::channel::<Option<String>>();
	
	let recording = record_value.is_some();

	let recorder = {
		let mut code = 0;
		let mut first = true;
		thread::spawn(move || {
			let mut recordFile = if let Some(s) = record_value {
				let mut file = std::fs::OpenOptions::new()
					.write(true)
					.append(false)
					.create(true)
					.truncate(true)
					.open(s);
				 match file {
					Ok(mut f) => { Some(f) },
					Err(e) => { eprintln!("{}", e); std::process::exit(1) }
				}
			} else {
				None
			};
			
			// TODO: Uncomment some of the commented sections below to 
			// record the state at each cycle. Also, comment the final
			// treatment of lastState.
			// This was changed to avoid writing to a file too much.
			// For replaying, the final state is all that is needed.
			// Adding extra information about the inputs and output
			// would be beneficial to reproduce the interactions
			// that led to the final state.
			
			// if let Some(file) = &mut recordFile {
			// 	file.write_all(b"[\n").expect("Writing to record file");
			// }
			let mut terminate = false;
			let mut lastState = None;
			while !terminate {
				let value = recv_data.recv();
				match value {
					Ok(v) => {
						match v {
							Some(state) => {
								lastState = Some(state);
								// if let Some(file) = &mut recordFile {
								// 	if !first {
								// 		file.write_all(b",\n").expect("Failed to write to record file");
								// 	} else {
								// 		first = false;
								// 	}
								// 	file.write_all(state.as_bytes()).expect("Failed to write to record file");
								// }
							},
							None => {
								terminate = true;
							}
						}

					},
					Err(e) => {
						terminate = true;
						code = 1;
						eprintln!("Failed to receive debug value.");
					}
				}
			}

			// if let Some(file) = &mut recordFile {
			// 	file.write_all(b"\n]\n").expect("Writing to record file");
			// }

			println!("\nTerminating application.");

			if let Some(file) = &mut recordFile {
				if let Some(state) = lastState {
					println!("Saving last state");
					file.write_all(state.as_bytes()).expect("Failed to write to record file");
				} else {
					println!("No state to save");
				}
				file.sync_all().expect("Failed to synchronise recorded file.");
			}

			std::process::exit(code);
		})
	};
	
	let ctrlc_send = send_data.clone();
	ctrlc::set_handler(move || {
		ctrlc_send.send(None).expect("Failed to send termination signal.");
	}).expect("Failed to setup ctrl-c handler.");
	
	// Module channel declarations;
	let (send_start_ReverseCmd, recv_start_ReverseCmd) = mpsc::sync_channel(0);
	let (send_end_ReverseCmd, recv_end_ReverseCmd) = mpsc::sync_channel(0);
	let (send_start_ctrl_ref0, recv_start_ctrl_ref0) = mpsc::sync_channel(0);
	let (send_end_ctrl_ref0, recv_end_ctrl_ref0) = mpsc::sync_channel(0);
	let (send_start_stm_ref0, recv_start_stm_ref0) = mpsc::sync_channel(0);
	let (send_end_stm_ref0, recv_end_stm_ref0) = mpsc::sync_channel(0);
	// Instantiate threads;
	let control_thread = {
	                       	thread::spawn(move || {
	                     		let mut terminate__: bool = false;
	                     		while !terminate__ {
	                     			{
	                     				let mut inputdone: bool = false;
	                     				while !inputdone {
	                     					{
	                     						let _s0: String;
	                     						_s0 = format!("{}", "Enter an event:");
	                     						print!("{}", _s0);
	                     						std::io::stdout().flush().unwrap();
	                     					}
	                     					let mut _event_: String;
	                     					let mut _temp_ = String::new();
	                     					std::io::stdin().read_line(&mut _temp_).expect("Failed to read input.");
	                     					_event_ = _temp_.as_str().trim().to_string();
	                     					match (_event_).as_str() {
	                     						"" => {
	                     							send_start_ReverseCmd.send(M_ReverseCmd_input::_done_).unwrap();
	                     							inputdone = true;
	                     						},
	                     						"opticalFlow" => {
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "Found event opticalFlow");
	                     								println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                     							}
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "Enter value for event opticalFlow of type real: ");
	                     								print!("{}", _s0);
	                     								std::io::stdout().flush().unwrap();
	                     							}
	                     							{
	                     								let mut _svalue_: String;
	                     								let mut _temp_ = String::new();
	                     								std::io::stdin().read_line(&mut _temp_).expect("Failed to read input.");
	                     								_svalue_ = _temp_.as_str().trim().to_string();
	                     								let mut _value_: f32 = 0.0_f32;
	                     								_value_ = (_svalue_).parse::<f32>().unwrap();
	                     								send_start_ReverseCmd.send(M_ReverseCmd_input::opticalFlow(_value_)).unwrap();
	                     							}
	                     						},
	                     						"MoveIn" => {
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "Found event MoveIn");
	                     								println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                     							}
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "Enter value for event MoveIn of type vector(real, 6): ");
	                     								print!("{}", _s0);
	                     								std::io::stdout().flush().unwrap();
	                     							}
	                     							{
	                     								let mut _value_: SVector<f32, 6> = (SVector::repeat(0.0_f32));
	                     								{
	                     									let _s0: String;
	                     									_s0 = format!("{}", "Enter value for repeated vector value  of type real: ");
	                     									print!("{}", _s0);
	                     									std::io::stdout().flush().unwrap();
	                     								}
	                     								{
	                     									let mut _svalue_: String;
	                     									let mut _temp_ = String::new();
	                     									std::io::stdin().read_line(&mut _temp_).expect("Failed to read input.");
	                     									_svalue_ = _temp_.as_str().trim().to_string();
	                     									let mut _base_value_: f32 = 0.0_f32;
	                     									_base_value_ = (_svalue_).parse::<f32>().unwrap();
	                     									_value_ = (SVector::repeat(_base_value_));
	                     								}
	                     								send_start_ReverseCmd.send(M_ReverseCmd_input::MoveIn(_value_)).unwrap();
	                     							}
	                     						},
	                     						"$end" => {
	                     							send_start_ReverseCmd.send(M_ReverseCmd_input::_terminate_).unwrap();
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "Terminating system. One more cycle will run before this program ends.");
	                     								println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                     							}
	                     							terminate__ = true;
	                     							inputdone = true;
	                     						},
	                     						_ => {
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", format!("{}{}","Unknown event: ", _event_));
	                     								println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                     							}
	                     						}
	                     					}
	                     				}
	                     			}
	                     			{
	                     				let mut outputdone: bool = false;
	                     				while !outputdone {
	                     					let mut _output_: M_ReverseCmd_output;
	                     					_output_ = recv_end_ReverseCmd.recv().unwrap();
	                     					match _output_ {
	                     						M_ReverseCmd_output::MoveOut(_aux1_) => {
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "output MoveOut");
	                     								print!("{}", _s0);
	                     								std::io::stdout().flush().unwrap();
	                     							}
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "(");
	                     								print!("{}", _s0);
	                     								std::io::stdout().flush().unwrap();
	                     							}
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", _aux1_);
	                     								print!("{}", _s0);
	                     								std::io::stdout().flush().unwrap();
	                     							}
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", ")");
	                     								println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                     							}
	                     						},
	                     						M_ReverseCmd_output::Land => {
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "output Land");
	                     								println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                     							}
	                     						},
	                     						M_ReverseCmd_output::_done_ => {
	                     							outputdone = true;
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "---------------------");
	                     								println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                     							}
	                     						},
	                     					}
	                     				}
	                     			}
	                     		}
	                     	})
	                     };		
	let mod_ReverseCmd_thread_thread = {
	                                     	thread::spawn(move || {
	                                   		let mut terminate__: bool = false;
	                                   		while !terminate__ {
	                                   			{
	                                   				let mut inputDone: bool = false;
	                                   				while !inputDone {
	                                   					{
	                                   						let _s0: String;
	                                   						_s0 = format!("{}", "- Waiting for input on channel start_ReverseCmd");
	                                   						debug!("{}", _s0);
	                                   					}
	                                   					let mut _input_: M_ReverseCmd_input;
	                                   					_input_ = recv_start_ReverseCmd.recv().unwrap();
	                                   					{
	                                   						let _s0: String;
	                                   						_s0 = format!("{}", "- Read input on channel start_ReverseCmd");
	                                   						debug!("{}", _s0);
	                                   					}
	                                   					match _input_ {
	                                   						M_ReverseCmd_input::MoveIn(_aux1_) => {
	                                   							send_start_ctrl_ref0.send(C_ctrl_ref0_input::MoveIn(_aux1_)).unwrap();
	                                   						},
	                                   						M_ReverseCmd_input::opticalFlow(_aux1_) => {
	                                   							send_start_ctrl_ref0.send(C_ctrl_ref0_input::opticalFlow(_aux1_)).unwrap();
	                                   						},
	                                   						M_ReverseCmd_input::_done_ => {
	                                   							send_start_ctrl_ref0.send(C_ctrl_ref0_input::_done_).unwrap();
	                                   							inputDone = true;
	                                   						},
	                                   						M_ReverseCmd_input::_terminate_ => {
	                                   							send_start_ctrl_ref0.send(C_ctrl_ref0_input::_terminate_).unwrap();
	                                   							terminate__ = true;
	                                   						},
	                                   					}
	                                   				}
	                                   			}
	                                   			{
	                                   				let _s0: String;
	                                   				_s0 = format!("{}", "Finished reading inputs of module ReverseCmd");
	                                   				debug!("{}", _s0);
	                                   			}
	                                   			mod_ReverseCmd_step(&send_start_ctrl_ref0
	                                   			                    , &recv_end_ctrl_ref0);
	                                   			{
	                                   				let mut outputDone: bool = false;
	                                   				while !outputDone {
	                                   					let mut _output_: C_ctrl_ref0_output;
	                                   					_output_ = recv_end_ctrl_ref0.recv().unwrap();
	                                   					match _output_ {
	                                   						C_ctrl_ref0_output::MoveOut(_aux1_) => {
	                                   							send_end_ReverseCmd.send(M_ReverseCmd_output::MoveOut(_aux1_)).unwrap();
	                                   						},
	                                   						C_ctrl_ref0_output::Land => {
	                                   							send_end_ReverseCmd.send(M_ReverseCmd_output::Land).unwrap();
	                                   						},
	                                   						C_ctrl_ref0_output::_done_ => {
	                                   							send_end_ReverseCmd.send(M_ReverseCmd_output::_done_).unwrap();
	                                   							outputDone = true;
	                                   						},
	                                   					}
	                                   				}
	                                   			}
	                                   		}
	                                   	})
	                                   };		
	let ctrl_ctrl_ref0_thread_thread = {
	                                     	thread::spawn(move || {
	                                   		let mut terminate__: bool = false;
	                                   		while !terminate__ {
	                                   			{
	                                   				let mut inputDone: bool = false;
	                                   				while !inputDone {
	                                   					{
	                                   						let _s0: String;
	                                   						_s0 = format!("{}", "- Waiting for input on channel start_ctrl_ref0");
	                                   						debug!("{}", _s0);
	                                   					}
	                                   					let mut _input_: C_ctrl_ref0_input;
	                                   					_input_ = recv_start_ctrl_ref0.recv().unwrap();
	                                   					{
	                                   						let _s0: String;
	                                   						_s0 = format!("{}", "- Read input on channel start_ctrl_ref0");
	                                   						debug!("{}", _s0);
	                                   					}
	                                   					match _input_ {
	                                   						C_ctrl_ref0_input::MoveIn(_aux1_) => {
	                                   							send_start_stm_ref0.send(stm_ref0_input::MoveIn(_aux1_)).unwrap();
	                                   						},
	                                   						C_ctrl_ref0_input::opticalFlow(_aux1_) => {
	                                   							send_start_stm_ref0.send(stm_ref0_input::opticalFlow(_aux1_)).unwrap();
	                                   						},
	                                   						C_ctrl_ref0_input::_done_ => {
	                                   							send_start_stm_ref0.send(stm_ref0_input::_done_).unwrap();
	                                   							inputDone = true;
	                                   						},
	                                   						C_ctrl_ref0_input::_terminate_ => {
	                                   							send_start_stm_ref0.send(stm_ref0_input::_terminate_).unwrap();
	                                   							terminate__ = true;
	                                   						},
	                                   					}
	                                   				}
	                                   			}
	                                   			{
	                                   				let _s0: String;
	                                   				_s0 = format!("{}", "	Finished reading inputs of controller ctrl_ref0");
	                                   				debug!("{}", _s0);
	                                   			}
	                                   			ctrl_ctrl_ref0_step(&send_start_stm_ref0
	                                   			                    , &recv_end_stm_ref0);
	                                   			{
	                                   				let mut outputDone: bool = false;
	                                   				while !outputDone {
	                                   					let mut _output_: stm_ref0_output;
	                                   					_output_ = recv_end_stm_ref0.recv().unwrap();
	                                   					match _output_ {
	                                   						stm_ref0_output::Land => {
	                                   							send_end_ctrl_ref0.send(C_ctrl_ref0_output::Land).unwrap();
	                                   						},
	                                   						stm_ref0_output::MoveOut(_aux1_) => {
	                                   							send_end_ctrl_ref0.send(C_ctrl_ref0_output::MoveOut(_aux1_)).unwrap();
	                                   						},
	                                   						stm_ref0_output::_done_ => {
	                                   							send_end_ctrl_ref0.send(C_ctrl_ref0_output::_done_).unwrap();
	                                   							outputDone = true;
	                                   						},
	                                   					}
	                                   				}
	                                   			}
	                                   		}
	                                   	})
	                                   };		
	let stm_stm_ref0_thread = {
	                          	let stm_stm_ref0_record = send_data.clone();
	                            	thread::spawn(move || {
	                          		// state machine variable declarations;
	                          		let mut inputstate: stm_ref0_inputstate = stm_ref0_inputstate {
	                          		                                          	MoveIn: false,
	                          		                                          	MoveIn_value: (SVector::repeat(0.0_f32)),
	                          		                                          	opticalFlow: false,
	                          		                                          	opticalFlow_value: 0.0_f32 as f32,
	                          		                                          	_clock_C: 0_i32,
	                          		                                          	_clock_T: 0_i32,
	                          		                                          	_transition_: TRANSITIONS_stm_ref0::NONE
	                          		                                          };
	                          		let mut state: stm_ref0_state = stm_ref0_state {
	                          		                                	done: false,
	                          		                                	state: STATES_stm_ref0::NONE,
	                          		                                	target_state: STATES_stm_ref0::NONE,
	                          		                                	status: STATUS::ENTER_STATE,
	                          		                                	en_ReverseCmdS_ReturnHome_1_done: false,
	                          		                                	en_ReverseCmdS_ReturnHome_1_counter: 0_i32,
	                          		                                	en_ReverseCmdS_SignalReceived_1_done: false,
	                          		                                	en_ReverseCmdS_SignalReceived_1_counter: 0_i32,
	                          		                                	tr_ReverseCmdS_t6_done: false,
	                          		                                	tr_ReverseCmdS_t6_counter: 0_i32,
	                          		                                	tr_ReverseCmdS_t2_done: false,
	                          		                                	tr_ReverseCmdS_t2_counter: 0_i32,
	                          		                                	tr_ReverseCmdS_t3_done: false,
	                          		                                	tr_ReverseCmdS_t3_counter: 0_i32,
	                          		                                	tr_ReverseCmdS_t0_done: false,
	                          		                                	tr_ReverseCmdS_t0_counter: 0_i32
	                          		                                };
	                          		let mut memorystate: stm_ref0_memory = stm_ref0_memory {
	                          		                                       	commands: (vec![]),
	                          		                                       	distanceToGround: 0.0_f32 as f32,
	                          		                                       	command: (SVector::repeat(0.0_f32)),
	                          		                                       	timeout: 1_i32 as f32,
	                          		                                       	i: 0_i32
	                          		                                       };
	                          		// state machine loop;
	                          		while !(state).done {
	                          			{
	                          				{
	                          					let _s0: String;
	                          					_s0 = format!("{}", "- Waiting for input on channel start_stm_ref0");
	                          					debug!("{}", _s0);
	                          				}
	                          				let mut inputDone: bool = false;
	                          				while !inputDone {
	                          					let mut _input_: stm_ref0_input;
	                          					_input_ = recv_start_stm_ref0.recv().unwrap();
	                          					{
	                          						let _s0: String;
	                          						_s0 = format!("{}", "- Read input on channel start_stm_ref0");
	                          						debug!("{}", _s0);
	                          					}
	                          					match _input_ {
	                          						stm_ref0_input::MoveIn(_aux_) => {
	                          							(inputstate).MoveIn = true;
	                          							(inputstate).MoveIn_value = _aux_;
	                          						},
	                          						stm_ref0_input::opticalFlow(_aux_) => {
	                          							(inputstate).opticalFlow = true;
	                          							(inputstate).opticalFlow_value = _aux_;
	                          						},
	                          						stm_ref0_input::_done_ => {
	                          							inputDone = true;
	                          						},
	                          						stm_ref0_input::_terminate_ => {
	                          							inputDone = true;
	                          						},
	                          					}
	                          				}
	                          			}
	                          			let mut ret: RESULT = RESULT::CONT;
	                          			while ret == RESULT::CONT {
	                          				let mut temp_: String;
	                          				temp_ = print_stm_ref0_state(&mut state);
	                          				{
	                          					let _s0: String;
	                          					_s0 = format!("{}", "	");
	                          					print!("{}", _s0);
	                          					std::io::stdout().flush().unwrap();
	                          				}
	                          				{
	                          					let _s0: String;
	                          					_s0 = format!("{}", temp_);
	                          					println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                          				}
	                          				ret = stm_stm_ref0_step(&mut state, &mut inputstate, &mut memorystate, &send_end_stm_ref0);
	                          			}
	                          			send_end_stm_ref0.send(stm_ref0_output::_done_).unwrap();
	                          			// update clocks;
	                          			(inputstate)._clock_C = ((inputstate)._clock_C + 1_i32);
	                          			(inputstate)._clock_T = ((inputstate)._clock_T + 1_i32);
	                          			// reset input events;
	                          			(inputstate).MoveIn = false;
	                          			(inputstate).opticalFlow = false;
	                          			{
	                          				let _s0: String;
	                          				_s0 = format!("{}", "		Sent output _done_ on channel end_stm_ref0");
	                          				debug!("{}", _s0);
	                          			}
	                          			{
	                          					let state_str = serde_json::to_string(&state).expect("Failed to serialise state of state machine stm_stm_ref0");
	                          					let memory_str = serde_json::to_string(&memorystate).expect("Failed to serialise memorystate of state machine stm_stm_ref0");
	                          				let snapshot = format!("{{\"state\": {},\"memory\": {}}}",state_str,memory_str);
	                          				stm_stm_ref0_record.send(Some(snapshot)).expect("Failed to send data to recorder."); 
	                          			}
	                          		}
	                          	})
	                          };		
	control_thread.join().unwrap();
	mod_ReverseCmd_thread_thread.join().unwrap();
	ctrl_ctrl_ref0_thread_thread.join().unwrap();
	stm_stm_ref0_thread.join().unwrap();
	
	println!("System successfully terminated.");
	return Ok(())
}

	fn print_STATUS(value:&mut STATUS)  -> String {
		match value {
			STATUS::ENTER_STATE => {
				return ("ENTER_STATE").to_string();
			},
			STATUS::ENTER_CHILDREN => {
				return ("ENTER_CHILDREN").to_string();
			},
			STATUS::EXECUTE_STATE => {
				return ("EXECUTE_STATE").to_string();
			},
			STATUS::EXIT_CHILDREN => {
				return ("EXIT_CHILDREN").to_string();
			},
			STATUS::EXIT_STATE => {
				return ("EXIT_STATE").to_string();
			},
			STATUS::INACTIVE => {
				return ("INACTIVE").to_string();
			},
		}
	}
	fn mod_ReverseCmd_step(send_start_ctrl_ref0: &SyncSender<C_ctrl_ref0_input>, recv_end_ctrl_ref0: &Receiver<C_ctrl_ref0_output>)  {
		{
			let _s0: String;
			_s0 = format!("{}", "Started step of module ReverseCmd");
			debug!("{}", _s0);
		}
		{
			let _s0: String;
			_s0 = format!("{}", "Finished step of module ReverseCmd");
			debug!("{}", _s0);
		}
	}
	fn print_STATES_stm_ref0(value:&mut STATES_stm_ref0)  -> String {
		match value {
			STATES_stm_ref0::NONE => {
				return ("NONE").to_string();
			},
			STATES_stm_ref0::SignalReceived => {
				return ("SignalReceived").to_string();
			},
			STATES_stm_ref0::ReturnHome => {
				return ("ReturnHome").to_string();
			},
			STATES_stm_ref0::Landing => {
				return ("Landing").to_string();
			},
			STATES_stm_ref0::final_f0 => {
				return ("final_f0").to_string();
			},
			STATES_stm_ref0::FailureMode => {
				return ("FailureMode").to_string();
			},
			STATES_stm_ref0::Waiting => {
				return ("Waiting").to_string();
			},
		}
	}
	fn tr_ReverseCmdS_t6(state:&mut stm_ref0_state, inputstate:&mut stm_ref0_inputstate, memory:&mut stm_ref0_memory, send_output: &SyncSender<stm_ref0_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running transition action of transition ReverseCmdS_t6.");
			debug!("{}", _s0);
		}
		if (state).tr_ReverseCmdS_t6_counter == 0_i32 {
			send_output.send(stm_ref0_output::MoveOut((memory).commands[(memory).i as usize])).unwrap();
			(*state).tr_ReverseCmdS_t6_counter = 1_i32;
			return RESULT::CONT;
		} else {
			(state).tr_ReverseCmdS_t6_done = true;
			return RESULT::CONT;
		}
	}
	fn tr_ReverseCmdS_t2(state:&mut stm_ref0_state, inputstate:&mut stm_ref0_inputstate, memory:&mut stm_ref0_memory, send_output: &SyncSender<stm_ref0_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running transition action of transition ReverseCmdS_t2.");
			debug!("{}", _s0);
		}
		if (state).tr_ReverseCmdS_t2_counter == 0_i32 {
			(*memory).i = ((memory).i + 1_i32);
			(*state).tr_ReverseCmdS_t2_counter = 1_i32;
			return RESULT::CONT;
		} else if (state).tr_ReverseCmdS_t2_counter == 1_i32 {
			(*memory).commands[(memory).i as usize] = (-1_i32 as f32 * (memory).command);
			(*state).tr_ReverseCmdS_t2_counter = 2_i32;
			return RESULT::CONT;
		} else {
			(state).tr_ReverseCmdS_t2_done = true;
			return RESULT::CONT;
		}
	}
	fn tr_ReverseCmdS_t0(state:&mut stm_ref0_state, inputstate:&mut stm_ref0_inputstate, memory:&mut stm_ref0_memory, send_output: &SyncSender<stm_ref0_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running transition action of transition ReverseCmdS_t0.");
			debug!("{}", _s0);
		}
		if (state).tr_ReverseCmdS_t0_counter == 0_i32 {
			send_output.send(stm_ref0_output::MoveOut((memory).commands[(memory).i as usize])).unwrap();
			(*state).tr_ReverseCmdS_t0_counter = 1_i32;
			return RESULT::CONT;
		} else {
			(state).tr_ReverseCmdS_t0_done = true;
			return RESULT::CONT;
		}
	}
	fn stm_stm_ref0_step(state:&mut stm_ref0_state, inputstate:&mut stm_ref0_inputstate, memorystate:&mut stm_ref0_memory, send_output: &SyncSender<stm_ref0_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "		Running step of state machine ReverseCmdS");
			debug!("{}", _s0);
		}
		match (*state).state {
			STATES_stm_ref0::NONE => {
				{
					let _s0: String;
					_s0 = format!("{}", "		Executing initial junction of ReverseCmdS");
					debug!("{}", _s0);
				}
				{
					(*state).state = STATES_stm_ref0::Waiting;
				}
				return RESULT::CONT;
			},
			STATES_stm_ref0::SignalReceived => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state SignalReceived");
							debug!("{}", _s0);
						}
						if !(state).en_ReverseCmdS_SignalReceived_1_done {
							let mut _ret_: RESULT;
							_ret_ = en_ReverseCmdS_SignalReceived_1(state, inputstate, memorystate, &send_output);
							return _ret_;
						} else {
							(*state).status = STATUS::ENTER_CHILDREN;
							(*state).en_ReverseCmdS_SignalReceived_1_done = false;
							(*state).en_ReverseCmdS_SignalReceived_1_counter = 0_i32;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state SignalReceived");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXECUTE_STATE;
						{
							(*inputstate)._transition_ = TRANSITIONS_stm_ref0::NONE;
						}
						return RESULT::CONT;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state SignalReceived");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_stm_ref0::NONE {
							if (((inputstate)._clock_T) as f32) < (((memorystate).timeout) as f32) {
								(*inputstate)._transition_ = TRANSITIONS_stm_ref0::stm_ref0_t2;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else if (((inputstate)._clock_T) as f32) >= (((memorystate).timeout) as f32) {
								(*inputstate)._transition_ = TRANSITIONS_stm_ref0::stm_ref0_t0;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else {
								return RESULT::CONT;
							}
						} else {
							return RESULT::CONT;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state SignalReceived");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXIT_STATE;
						return RESULT::CONT;
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state SignalReceived");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_stm_ref0::stm_ref0_t2 {
								if !(state).tr_ReverseCmdS_t2_done {
									let mut _ret_: RESULT;
									_ret_ = tr_ReverseCmdS_t2(state, inputstate, memorystate, &send_output);
									return _ret_;
								} else {
									(*state).state = STATES_stm_ref0::SignalReceived;
									(*state).status = STATUS::ENTER_STATE;
									(*state).tr_ReverseCmdS_t2_done = false;
									(*state).tr_ReverseCmdS_t2_counter = 0_i32;
									return RESULT::CONT;
								}
							} else if (*inputstate)._transition_ == TRANSITIONS_stm_ref0::stm_ref0_t0 {
								if !(state).tr_ReverseCmdS_t0_done {
									let mut _ret_: RESULT;
									_ret_ = tr_ReverseCmdS_t0(state, inputstate, memorystate, &send_output);
									return _ret_;
								} else {
									(*state).state = STATES_stm_ref0::ReturnHome;
									(*state).status = STATUS::ENTER_STATE;
									(*state).tr_ReverseCmdS_t0_done = false;
									(*state).tr_ReverseCmdS_t0_counter = 0_i32;
									return RESULT::CONT;
								}
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_stm_ref0::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State SignalReceived is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
			STATES_stm_ref0::ReturnHome => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state ReturnHome");
							debug!("{}", _s0);
						}
						if !(state).en_ReverseCmdS_ReturnHome_1_done {
							let mut _ret_: RESULT;
							_ret_ = en_ReverseCmdS_ReturnHome_1(state, inputstate, memorystate, &send_output);
							return _ret_;
						} else {
							(*state).status = STATUS::ENTER_CHILDREN;
							(*state).en_ReverseCmdS_ReturnHome_1_done = false;
							(*state).en_ReverseCmdS_ReturnHome_1_counter = 0_i32;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state ReturnHome");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXECUTE_STATE;
						{
							(*inputstate)._transition_ = TRANSITIONS_stm_ref0::NONE;
						}
						return RESULT::CONT;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state ReturnHome");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_stm_ref0::NONE {
							if (((memorystate).i) as f32) <= ((0_i32) as f32) && (inputstate).opticalFlow {
								(*memorystate).distanceToGround = (inputstate).opticalFlow_value;
								(*inputstate)._transition_ = TRANSITIONS_stm_ref0::stm_ref0_t3;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else if (((memorystate).i) as f32) > ((0_i32) as f32) {
								(*inputstate)._transition_ = TRANSITIONS_stm_ref0::stm_ref0_t6;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::CONT;
							} else {
								return RESULT::CONT;
							}
						} else {
							return RESULT::CONT;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state ReturnHome");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXIT_STATE;
						return RESULT::CONT;
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state ReturnHome");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_stm_ref0::stm_ref0_t3 {
								if !(state).tr_ReverseCmdS_t3_done {
									let mut _ret_: RESULT;
									_ret_ = tr_ReverseCmdS_t3(state, inputstate, memorystate, &send_output);
									return _ret_;
								} else {
									(*state).state = STATES_stm_ref0::Landing;
									(*state).status = STATUS::ENTER_STATE;
									(*state).tr_ReverseCmdS_t3_done = false;
									(*state).tr_ReverseCmdS_t3_counter = 0_i32;
									return RESULT::CONT;
								}
							} else if (*inputstate)._transition_ == TRANSITIONS_stm_ref0::stm_ref0_t6 {
								if !(state).tr_ReverseCmdS_t6_done {
									let mut _ret_: RESULT;
									_ret_ = tr_ReverseCmdS_t6(state, inputstate, memorystate, &send_output);
									return _ret_;
								} else {
									(*state).state = STATES_stm_ref0::ReturnHome;
									(*state).status = STATUS::ENTER_STATE;
									(*state).tr_ReverseCmdS_t6_done = false;
									(*state).tr_ReverseCmdS_t6_counter = 0_i32;
									return RESULT::CONT;
								}
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_stm_ref0::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State ReturnHome is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
			STATES_stm_ref0::Landing => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state Landing");
							debug!("{}", _s0);
						}
						{
							(*state).status = STATUS::ENTER_CHILDREN;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state Landing");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXECUTE_STATE;
						{
							(*inputstate)._transition_ = TRANSITIONS_stm_ref0::NONE;
						}
						return RESULT::CONT;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state Landing");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_stm_ref0::NONE {
							if (inputstate).opticalFlow && (((memorystate).distanceToGround) as f32) >= ((0.1_f32) as f32) {
								(*memorystate).distanceToGround = (inputstate).opticalFlow_value;
								(*inputstate)._transition_ = TRANSITIONS_stm_ref0::stm_ref0_t5;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else if (((memorystate).distanceToGround) as f32) < ((0.1_f32) as f32) {
								(*inputstate)._transition_ = TRANSITIONS_stm_ref0::stm_ref0_t4;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::CONT;
							} else if true {
								(*inputstate)._transition_ = TRANSITIONS_stm_ref0::stm_ref0_t7;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::CONT;
							} else {
								return RESULT::CONT;
							}
						} else {
							return RESULT::CONT;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state Landing");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXIT_STATE;
						return RESULT::CONT;
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state Landing");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_stm_ref0::stm_ref0_t5 {
								(*state).state = STATES_stm_ref0::Landing;
								(*state).status = STATUS::ENTER_STATE;
								return RESULT::CONT;
							} else if (*inputstate)._transition_ == TRANSITIONS_stm_ref0::stm_ref0_t4 {
								(*state).state = STATES_stm_ref0::final_f0;
								(*state).status = STATUS::ENTER_STATE;
								return RESULT::CONT;
							} else if (*inputstate)._transition_ == TRANSITIONS_stm_ref0::stm_ref0_t7 {
								(*state).state = STATES_stm_ref0::FailureMode;
								(*state).status = STATUS::ENTER_STATE;
								return RESULT::CONT;
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_stm_ref0::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State Landing is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
			STATES_stm_ref0::final_f0 => {
				{
					let _s0: String;
					_s0 = format!("{}", "		Entering final state f0");
					debug!("{}", _s0);
				}
				return RESULT::CONT;
			},
			STATES_stm_ref0::FailureMode => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state FailureMode");
							debug!("{}", _s0);
						}
						{
							(*state).status = STATUS::ENTER_CHILDREN;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state FailureMode");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXECUTE_STATE;
						{
							(*inputstate)._transition_ = TRANSITIONS_stm_ref0::NONE;
						}
						return RESULT::CONT;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state FailureMode");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_stm_ref0::NONE {
							if true {
								(*inputstate)._transition_ = TRANSITIONS_stm_ref0::stm_ref0_t8;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::CONT;
							} else {
								return RESULT::CONT;
							}
						} else {
							return RESULT::CONT;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state FailureMode");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXIT_STATE;
						return RESULT::CONT;
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state FailureMode");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_stm_ref0::stm_ref0_t8 {
								(*state).state = STATES_stm_ref0::Landing;
								(*state).status = STATUS::ENTER_STATE;
								return RESULT::CONT;
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_stm_ref0::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State FailureMode is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
			STATES_stm_ref0::Waiting => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state Waiting");
							debug!("{}", _s0);
						}
						{
							(*state).status = STATUS::ENTER_CHILDREN;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state Waiting");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXECUTE_STATE;
						{
							(*inputstate)._transition_ = TRANSITIONS_stm_ref0::NONE;
						}
						return RESULT::CONT;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state Waiting");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_stm_ref0::NONE {
							if !(inputstate).MoveIn {
								(*memorystate).command = (inputstate).MoveIn_value;
								(*inputstate)._transition_ = TRANSITIONS_stm_ref0::stm_ref0_t1b;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::CONT;
							} else if (inputstate).MoveIn {
								(*memorystate).command = (inputstate).MoveIn_value;
								(*inputstate)._transition_ = TRANSITIONS_stm_ref0::stm_ref0_t1a;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::CONT;
							} else {
								return RESULT::CONT;
							}
						} else {
							return RESULT::CONT;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state Waiting");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXIT_STATE;
						return RESULT::CONT;
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state Waiting");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_stm_ref0::stm_ref0_t1b {
								(*state).state = STATES_stm_ref0::Waiting;
								(*state).status = STATUS::ENTER_STATE;
								return RESULT::CONT;
							} else if (*inputstate)._transition_ == TRANSITIONS_stm_ref0::stm_ref0_t1a {
								(*state).state = STATES_stm_ref0::SignalReceived;
								(*state).status = STATUS::ENTER_STATE;
								return RESULT::CONT;
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_stm_ref0::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State Waiting is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
		}
	}
	fn ctrl_ctrl_ref0_step(send_start_stm_ref0: &SyncSender<stm_ref0_input>, recv_end_stm_ref0: &Receiver<stm_ref0_output>)  {
		{
			let _s0: String;
			_s0 = format!("{}", "	Started step of controller ctrl_ref0");
			debug!("{}", _s0);
		}
	}
	fn print_stm_ref0_state(state:&mut stm_ref0_state)  -> String {
		let mut temp1_: String;
		temp1_ = print_STATES_stm_ref0(&mut (state).state);
		let mut temp2_: String;
		temp2_ = print_STATUS(&mut (state).status);
		return format!("{}{}",format!("{}{}",format!("{}{}",temp1_, " ("), temp2_), ")");
	}
	fn tr_ReverseCmdS_t3(state:&mut stm_ref0_state, inputstate:&mut stm_ref0_inputstate, memory:&mut stm_ref0_memory, send_output: &SyncSender<stm_ref0_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running transition action of transition ReverseCmdS_t3.");
			debug!("{}", _s0);
		}
		if (state).tr_ReverseCmdS_t3_counter == 0_i32 {
			send_output.send(stm_ref0_output::Land).unwrap();
			(*state).tr_ReverseCmdS_t3_counter = 1_i32;
			return RESULT::CONT;
		} else {
			(state).tr_ReverseCmdS_t3_done = true;
			return RESULT::CONT;
		}
	}
	fn en_ReverseCmdS_ReturnHome_1(state:&mut stm_ref0_state, inputstate:&mut stm_ref0_inputstate, memory:&mut stm_ref0_memory, send_output: &SyncSender<stm_ref0_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running entry action 1 of state ReverseCmdS_ReturnHome.");
			debug!("{}", _s0);
		}
		if (state).en_ReverseCmdS_ReturnHome_1_counter == 0_i32 {
			(*memory).i = ((memory).i - 1_i32);
			(*state).en_ReverseCmdS_ReturnHome_1_counter = 1_i32;
			return RESULT::CONT;
		} else {
			(state).en_ReverseCmdS_ReturnHome_1_done = true;
			return RESULT::CONT;
		}
	}
	fn en_ReverseCmdS_SignalReceived_1(state:&mut stm_ref0_state, inputstate:&mut stm_ref0_inputstate, memory:&mut stm_ref0_memory, send_output: &SyncSender<stm_ref0_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running entry action 1 of state ReverseCmdS_SignalReceived.");
			debug!("{}", _s0);
		}
		if (state).en_ReverseCmdS_SignalReceived_1_counter == 0_i32 {
			(*inputstate)._clock_T = 0_i32;
			(*state).en_ReverseCmdS_SignalReceived_1_counter = 1_i32;
			return RESULT::CONT;
		} else {
			(state).en_ReverseCmdS_SignalReceived_1_done = true;
			return RESULT::CONT;
		}
	}



