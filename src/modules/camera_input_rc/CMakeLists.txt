############################################################################
px4_add_module(
	MODULE drivers__camera_input_rc
	MAIN camera_input_rc
	COMPILE_FLAGS
	SRCS
		camera_input_rc.cpp
	)
