	pid_controller u0 (
		.address     (<connected-to-address>),     // avalon_slave_0.address
		.write       (<connected-to-write>),       //               .write
		.writedata   (<connected-to-writedata>),   //               .writedata
		.read        (<connected-to-read>),        //               .read
		.readdata    (<connected-to-readdata>),    //               .readdata
		.waitrequest (<connected-to-waitrequest>), //               .waitrequest
		.clock       (<connected-to-clock>),       //     clock_sink.clk
		.o_output    (<connected-to-o_output>),    //    conduit_end.export
		.reset       (<connected-to-reset>)        //          reset.reset
	);

