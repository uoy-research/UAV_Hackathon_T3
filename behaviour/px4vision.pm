pmodel px4vision {
	local link base_link {
		def {
			inertial information {
				mass 1.5
				inertia matrix { ixx 0.029125 ixy 0 ixz 0 iyy 0.029125 iyz 0 izz 0.055225 }
				pose {
					x = 0
					y = 0
					z = 0
					roll = 0
					pitch = 0
					yaw = 0
				}
			}
		}
		local body base_link_inertia_collision {
			def {
				box ( length = 0.47 , width = 0.47 , height = 0.11 )
			}
			pose {
				x = 0
				y = 0
				z = 0
				roll = 0
				pitch = 0
				yaw = 0
			}
		}
		local body base_link_inertia_visual {
			def {
			}
			pose {
				x = 0
				y = 0
				z = 0
				roll = 1.5707
				pitch = 0
				yaw = - 0.01
			}
		}
		local joint imu_joint {
			def {
			}
			flexibly connected to imu_link
			pose {
				x = 0
				y = 0
				z = 0
				roll = 0
				pitch = 0
				yaw = 0
			}
		}
		local joint rotor1 {
			def {
			}
			flexibly connected to rotor1
			pose {
				x = 0
				y = 0
				z = 0
				roll = 0
				pitch = 0
				yaw = 0
			}
		local actuator back_left_motor {
				def {
				}
				pose {
					x = 0
					y = 0
					z = 0
					roll = 0
					pitch = 0
					yaw = 0
				}
			}
		}
		local joint rotor2 {
			def {
			}
			flexibly connected to rotor_2
			pose {
				x = 0
				y = 0
				z = 0
				roll = 0
				pitch = 0
				yaw = 0
			}
		local actuator front_left_motor {
				def {
				}
				pose {
					x = 0
					y = 0
					z = 0
					roll = 0
					pitch = 0
					yaw = 0
				}
			}
		}
		local joint rotor3 {
			def {
			}
			flexibly connected to rotor_3
			pose {
				x = 0
				y = 0
				z = 0
				roll = 0
				pitch = 0
				yaw = 0
			}
		local actuator back_right_motor {
				def {
				}
				pose {
					x = 0
					y = 0
					z = 0
					roll = 0
					pitch = 0
					yaw = 0
				}
			}
		}
		local joint rotor0 {
			def {
			}
			flexibly connected to rotor_0
			pose {
				x = 0
				y = 0
				z = 0
				roll = 0
				pitch = 0
				yaw = 0
			}
		local actuator front_right_motor {
				def {
				}
				pose {
					x = 0
					y = 0
					z = 0
					roll = 0
					pitch = 0
					yaw = 0
				}
			}
		}
		pose {
			x = 0
			y = 0
			z = 0
			roll = 0
			pitch = 0
			yaw = 0
		}
	}
	local link imu_link {
		def {
		}
		pose {
			x = 0
			y = 0
			z = 0
			roll = 0
			pitch = 0
			yaw = 0
		}
	}
	local link rotor1 {
		def {
		}
		pose {
			x = - 0.0935
			y = 0.107
			z = 0.03
			roll = 0
			pitch = 0
			yaw = 0
		}
	}
	local link rotor_2 {
		def {
		}
		pose {
			x = 0.0935
			y = 0.107
			z = 0.03
			roll = 0
			pitch = 0
			yaw = 0
		}
	}
	local link rotor_3 {
		def {
		}
		pose {
			x = - 0.0935
			y = - 0.107
			z = 0.03
			roll = 0
			pitch = 0
			yaw = 0
		}
	}
	local link rotor_0 {
		def {
		}
		pose {
			x = 0.0935
			y = - 0.107
			z = 0.03
			roll = 0
			pitch = 0
			yaw = 0
		}
	}
}
