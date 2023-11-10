
interface CommandIn {
	event MoveIn: vector(real,6)
}

interface CommandOut {
	MoveOut( move: vector(real,6)  )
	
	
}
interface SensorIn {
	event opticalFlow: real
	event RGBD: vector(real, 4)
	event RGBD2: vector(real, 4)
}

interface OA{
	event ResetCommandOut: int
	event obstacle
	event landing 
	event StartLocalisation
}

interface LM{
	event PoseEstimate: vector(real,6)
}

interface RCmd{
	event Publish_t:int
	event PublishCommands: Seq(vector(real,6))
}
controller ReverseCmdC {
	uses CommandIn uses SensorIn requires CommandOut sref stm_ref0 = ReverseCmdS
	connection ReverseCmdC on MoveIn to stm_ref0 on MoveIn
	cycleDef cycle == 1
	connection ReverseCmdC on opticalFlow to stm_ref0 on opticalFlow

sref stm_ref1 = ObstacleAvoidance
	sref stm_ref2 = LocalisationAndMapping
connection stm_ref1 on landing to stm_ref0 on landing
	connection stm_ref1 on ResetCommandOut to stm_ref0 on ResetCommandOut
	connection stm_ref1 on obstacle to stm_ref0 on obstacle
	connection stm_ref2 on PoseEstimate to stm_ref1 on PoseEstimate

	connection ReverseCmdC on RGBD to stm_ref1 on RGBD
	connection ReverseCmdC on RGBD2 to stm_ref2 on RGBD2
connection stm_ref1 on Publish_t to stm_ref0 on Publish_t
	connection stm_ref1 on PublishCommands to stm_ref0 on PublishCommands
connection stm_ref1 on StartLocalisation to stm_ref2 on StartLocalisation
}

stm LocalisationAndMapping {
	input context {uses SensorIn uses OA}
	output context {uses LM }
	cycleDef cycle == 1
	state Mapping {
	}
	initial i0
	state Localisation {
	}
	transition t0 {
		from i0
		to Mapping
	}
transition t1 {
		from Mapping
		to Localisation		
		exec
		condition $StartLocalisation
	}
}

stm ObstacleAvoidance {
	input context {uses SensorIn uses RCmd uses LM}
	output context {uses OA }
	cycleDef cycle == 1
	initial i0
	state s0 {
	}
	transition t0 {
		from i0
		to s0
	}
}


stm ReverseCmdS {
	clock C
	clock T
	const timeout: real = 1
	var distanceToGround: real
	var command: vector(real,6)
	var commands: Seq(vector(real,6))
	var new_commands: Seq(vector(real,6))
	var i: int = 0
	var t: int = 0
	var new_t: int
	const stop : vector(real,6) = (|0.0, 0.0, 0.0, 0.0, 0.0, 0.0|)
	const rotate: vector(real, 6) = (|0.0,0.0,0.0,0.0,0.0,1.0|)
	const land: vector(real, 6) = (|0.0,0.0,-0.2,0.0,0.0,0.0|)
	var tmax : int = 0

	input context { uses CommandIn uses SensorIn uses OA}
	output context {requires CommandOut uses RCmd}
	cycleDef cycle == 1
initial i0
	state SignalReceived {
		entry #T
	}
	state ReturnHome {
		entry t = t+1
	}
	state Landing {
	}
	final f0

	state Waiting {}
	
	state Rotate {
	}
	state ObstacleAvoidance {
	}
	state WaitForSignal {
	}
	state ReverseCommands {
		entry i = i-1; t = t+1
	}

	transition t1 {
		from i0
		to Waiting
	}
	
	transition t1a {
		from Waiting
		to SignalReceived
		condition $MoveIn?command
	}
	
	transition t1b {
		from Waiting
		to Waiting
		exec
		condition not $MoveIn?command
	}
	
	transition t2 {
		from WaitForSignal
		to SignalReceived
		condition since ( T ) < timeout /\ $MoveIn?command
		action i = i + 1; commands[i] = command
	}
transition t0 {
		from WaitForSignal
		to Rotate
		exec
		condition since ( T ) >= timeout
		action CommandOut::MoveOut(stop); #T; tmax = i
	}
	transition t3 {
		from ReturnHome
		to Landing
		exec
		condition t <= tmax /\ $opticalFlow?distanceToGround /\ not $obstacle
		action CommandOut::MoveOut(land)
		
	}
	transition t4 {
		from Landing
		to f0
		condition distanceToGround < 0.1
	}
transition t5 {
		from Landing
		to Landing
		exec
		condition $opticalFlow?distanceToGround /\ distanceToGround >= 0.1
		action CommandOut::MoveOut(land)
		
	}
transition t6 {
		from ReturnHome
		to ReturnHome
		condition t > 0 /\ not $obstacle
		action CommandOut::MoveOut(commands[t])
	}
	transition t9 {
		from Rotate
		to ReverseCommands
		exec
		condition since(T) > 3.14
		action CommandOut::MoveOut(stop) 
	}
	transition t7 {
		from ReturnHome
		to ObstacleAvoidance
		exec
		condition $obstacle
		action CommandOut::MoveOut(stop); $Publish_t!t
	}
	transition t8 {
		from ObstacleAvoidance
		to ReturnHome
		exec
		condition  
		$ ResetCommandOut ? new_t
		action t = new_t
	}
	transition t10 {
		from WaitForSignal
		to WaitForSignal
		exec
		condition since (T) < timeout /\ not $MoveIn?command
	}
transition t11 {
		from Rotate
		to Rotate
		exec
		condition since(T) <= 3.14
		action CommandOut::MoveOut(rotate)
	}
transition t12 {
		from SignalReceived
		to WaitForSignal
		exec
	}
transition t13 {
		from ObstacleAvoidance
		to ObstacleAvoidance
		exec
		condition not $ ResetCommandOut
	}
transition t14 {
		from ObstacleAvoidance
		to Landing
		exec 
		condition $landing
		action CommandOut::MoveOut(land)
	}
transition t15 {
		from ReverseCommands
		to ReturnHome
		action $PublishCommands!commands; t = 0; CommandOut::MoveOut(new_commands[t])
	}
transition t16 {
		from ReverseCommands
		to ReverseCommands
		action new_commands[t] = (|(commands[i])[0],(commands[i])[1],(-commands[i])[2],(commands[i])[3],-(commands[i])[4],(commands[i])[5]|)
	}
}



module ReverseCmd {
	robotic platform px4vision {
		 uses SensorIn uses CommandIn provides CommandOut }

	cref ctrl_ref0 = ReverseCmdC
	cycleDef cycle == 1

	connection px4vision on MoveIn to ctrl_ref0 on MoveIn ( _async )
connection px4vision on opticalFlow to ctrl_ref0 on opticalFlow ( _async )

	connection px4vision on RGBD to ctrl_ref0 on RGBD ( _async )
	connection px4vision on RGBD2 to ctrl_ref0 on RGBD2 ( _async )
}

