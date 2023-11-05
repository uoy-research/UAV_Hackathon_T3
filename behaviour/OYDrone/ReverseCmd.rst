
interface CommandIn {
	event MoveIn: vector(real,6)
	event ResetCommandOut: int
}

interface CommandOut {
	MoveOut( move: vector(real,6)  )
	PublishCommands(vec: Seq(vector(real,6)))
	Publish_t(t:int)
}
interface SensorIn {
	event opticalFlow: real
	event obstacle
	event landing 
}
controller ReverseCmdC {
	uses CommandIn uses SensorIn requires CommandOut sref stm_ref0 = ReverseCmdS
	
	connection ReverseCmdC on MoveIn to stm_ref0 on MoveIn

	cycleDef cycle == 1
connection ReverseCmdC on opticalFlow to stm_ref0 on opticalFlow
connection ReverseCmdC on ResetCommandOut to stm_ref0 on ResetCommandOut
	connection ReverseCmdC on obstacle to stm_ref0 on obstacle
	connection ReverseCmdC on landing to stm_ref0 on landing
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

	input context { uses CommandIn uses SensorIn}
	output context {requires CommandOut}
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
	state LocalReplanner {
		}
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
		action CommandOut::MoveOut(stop); #T
	}
	transition t3 {
		from ReturnHome
		to Landing
		exec
		condition t <= 0 /\ $opticalFlow?distanceToGround /\ not $obstacle
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
		action CommandOut::MoveOut(stop); CommandOut::Publish_t(t)
	}
	transition t8 {
		from ObstacleAvoidance
		to ReturnHome
		exec
		condition $ResetCommandOut?new_t /\ not $obstacle 
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
		condition $obstacle
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
		action CommandOut::PublishCommands(commands); t = 0; CommandOut::MoveOut(new_commands[t])
	}
transition t16 {
		from ReverseCommands
		to ReverseCommands
		action new_commands[t] = (|(commands[i])[0],(commands[i])[1],(-commands[i])[2],(commands[i])[3],-(commands[i])[4],(commands[i])[5]|)
	}
}

module ReverseCmd {
	robotic platform px4vision {
		uses CommandIn uses SensorIn provides CommandOut }

	cref ctrl_ref0 = ReverseCmdC
	cycleDef cycle == 1

	connection px4vision on MoveIn to ctrl_ref0 on MoveIn ( _async )
connection px4vision on opticalFlow to ctrl_ref0 on opticalFlow ( _async )
	connection px4vision on ResetCommandOut to ctrl_ref0 on ResetCommandOut ( _async )
	connection px4vision on obstacle to ctrl_ref0 on obstacle ( _async )
	connection px4vision on landing to ctrl_ref0 on landing ( _async )
}

