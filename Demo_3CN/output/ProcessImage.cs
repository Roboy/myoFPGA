using System;
using System.Runtime.InteropServices;
/// <summary>
/// This file was autogenerated by openCONFIGURATOR-1.4.1 on 10-Jun-2016 19:23:59
/// </summary>

namespace openPOWERLINK
{

	/// <summary>
	/// Struct : ProcessImage Out
	/// </summary>
	[StructLayout(LayoutKind.Explicit, Pack = 1, Size = 4)]
	public struct AppProcessImageOut
	{
		[FieldOffset(0)]
		public byte CN1_M00_DigitalInput_00h_AU8_DigitalInput;
		[FieldOffset(1)]
		public byte CN32_M00_DigitalInput_00h_AU8_DigitalInput;
		[FieldOffset(2)]
		public byte CN110_M00_DigitalInput_00h_AU8_DigitalInput;
		[FieldOffset(3)]
		public byte PADDING_VAR_1;
	}

	/// <summary>
	/// Struct : ProcessImage In
	/// </summary>
	[StructLayout(LayoutKind.Explicit, Pack = 1, Size = 4)]
	public struct AppProcessImageIn
	{
		[FieldOffset(0)]
		public byte CN1_M00_DigitalOutput_00h_AU8_DigitalOutput;
		[FieldOffset(1)]
		public byte CN32_M00_DigitalOutput_00h_AU8_DigitalOutput;
		[FieldOffset(2)]
		public byte CN110_M00_DigitalOutput_00h_AU8_DigitalOutput;
		[FieldOffset(3)]
		public byte PADDING_VAR_1;
	}
}