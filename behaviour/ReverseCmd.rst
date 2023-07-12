
interface CommandIn {
	event MoveIn: vector(real,6)
}

interface CommandOut {
	MoveOut( move: vector(real,6)  )
	Land()
}
interface SensorIn {
	event opticalFlow: real
}
controller ReverseCmdC {
	uses CommandIn sref stm_ref0 = ReverseCmdS
	
	connection ReverseCmdC on MoveIn to stm_ref0 on MoveIn

	cycleDef cycle == 1
}

stm ReverseCmdS {
	clock C
	clock T
	const timeout: real = 1
	var distanceToGround: real
	var command: vector(real,6)
	var commands: Seq(vector(real,6))
	var i: int

	input context { uses CommandIn uses SensorIn}
	output context {requires CommandOut}
	cycleDef cycle == 1
initial i0
	state SignalReceived {
		entry #T
	}
	state ReturnHome {
		entry i = i-1
	}
	state Landing {
	}
	final f0
	state FailureMode {
	}

	transition t1 {
		from i0
		to SignalReceived
		condition $MoveIn?command
	}
	transition t2 {
		from SignalReceived
		to SignalReceived
		exec
		condition since ( T ) < timeout
		action i = i + 1; commands[i] = -1*command
	}
transition t0 {
		from SignalReceived
		to ReturnHome
		exec
		condition since ( T ) >= timeout
		action CommandOut::MoveOut(commands[i])
	}
	transition t3 {
		from ReturnHome
		to Landing
		exec
		condition i <= 0 /\ $opticalFlow?distanceToGround
		action CommandOut::Land()
		
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
		
	}
transition t6 {
		from ReturnHome
		to ReturnHome
		condition i > 0
		action CommandOut::MoveOut(commands[i])
	}
transition t7 {
		from Landing
		to FailureMode
	}
	transition t8 {
		from FailureMode
		to Landing
	}
}

module ReverseCmd {
	robotic platform px4vision {
		uses CommandIn provides CommandOut }

	cref ctrl_ref0 = ReverseCmdC
	cycleDef cycle == 1

	connection px4vision on MoveIn to ctrl_ref0 on MoveIn ( _async )
}

