import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, Timer, ClockCycles


expected_output = [b'Aeonic',
                   b'000A: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00',
                   b'02F2: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00',
                   b'000A: 53 74 72 69 6E 67 31 20 12 34 56 78 9A BC DE F0',
                   b'02F2: 88 77 66 55 44 33 22 11 53 74 72 69 6E 67 31 20',
                   b'AA 55 55 AA AA 55 55 AA',
                   b'0179',
                   b'04',
                   b'07']

@cocotb.test()
async def test_aeonic_tt(dut):
    dut._log.info("start")
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())

    expected_output_index = 0

    gp_out_state = False
    output_str = b''

    cycle = 0

    dut._log.info("reset")
    dut.rst.value = 1
    for i in range(10):
        await FallingEdge(dut.clk)
    dut.rst.value = 0

    while True:
        await FallingEdge(dut.clk)
        if False:
            print(f'{cycle=} dut.gp_out_stb={str(dut.gp_out_stb)} dut.gp_out={str(dut.gp_out)}')
        if dut.gp_out_stb == 1:
            nib = dut.gp_out.value
            if not gp_out_state:
                high_nib = nib
            else:
                byte = (high_nib << 4) | nib
                if byte == 0x04:
                    break
                elif byte == 0x0a:
                    print(str(output_str, "ascii"))
                    assert(output_str == expected_output[expected_output_index])
                    expected_output_index += 1
                    output_str = b''
                else:
                    output_str = output_str + bytes([byte])
            gp_out_state = not gp_out_state
        cycle += 1

    assert(expected_output_index == len(expected_output))

