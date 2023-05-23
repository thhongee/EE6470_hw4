#ifndef SOBEL_FILTER_H_
#define SOBEL_FILTER_H_
#include <cmath>
#include <iomanip>
#include <systemc>
using namespace sc_core;

#include <tlm_utils/simple_target_socket.h>

#include <tlm>

#include "filter_def.h"

struct SobelFilter : public sc_module {
	tlm_utils::simple_target_socket<SobelFilter> tsock;

	sc_fifo<unsigned char> i_r;
	sc_fifo<unsigned char> i_g;
	sc_fifo<unsigned char> i_b;
	sc_fifo<int> o_result;

	SC_HAS_PROCESS(SobelFilter);

	SobelFilter(sc_module_name n) : sc_module(n), tsock("t_skt"), base_offset(0) {
		tsock.register_b_transport(this, &SobelFilter::blocking_transport);
		SC_THREAD(do_filter);
	}

	~SobelFilter() {}

	unsigned int base_offset;

	void do_filter() {
		unsigned int a[9], buf[9];
		{ wait(CLOCK_PERIOD, SC_NS); }
		while (true) {
			for (unsigned int v = 0; v < 9; ++v) {
				unsigned char grey = (i_r.read() + i_g.read() + i_b.read()) / 3;
				a[v] = grey;
				wait(CLOCK_PERIOD, SC_NS);
			}

			for (int i = 0; i < 9; i++) {
				buf[i] = a[i];
				wait(CLOCK_PERIOD, SC_NS);
			}

			// sorting
			{
				bool swapped = true;
				int j = 0;

				while (swapped) {
					swapped = false;
					j++;
					for (int i = 0; i < 9 - j; ++i) {
						if (buf[i] > buf[i + 1]) {
							int temp = buf[i];
							buf[i] = buf[i + 1];
							buf[i + 1] = temp;
							swapped = true;
							wait(CLOCK_PERIOD, SC_NS);
						}
					}
				}
			}
			a[4] = buf[4];

			// Mean filter
			double total = 0;
			int k = 0;

			for (unsigned int v = 0; v < MASK_Y; ++v) {
				for (unsigned int u = 0; u < MASK_X; ++u) {
					total += a[k] * mask[v][u];
					k++;
					wait(CLOCK_PERIOD, SC_NS);
				}
			}

			int result = static_cast<int>(total / 10);

			// cout << (int)result << endl;

			o_result.write(result);
		}
	}

	void blocking_transport(tlm::tlm_generic_payload &payload, sc_core::sc_time &delay) {
		wait(delay);
		// unsigned char *mask_ptr = payload.get_byte_enable_ptr();
		// auto len = payload.get_data_length();
		tlm::tlm_command cmd = payload.get_command();
		sc_dt::uint64 addr = payload.get_address();
		unsigned char *data_ptr = payload.get_data_ptr();

		addr -= base_offset;

		// cout << (int)data_ptr[0] << endl;
		// cout << (int)data_ptr[1] << endl;
		// cout << (int)data_ptr[2] << endl;
		word buffer;

		switch (cmd) {
			case tlm::TLM_READ_COMMAND:
				// cout << "READ" << endl;
				switch (addr) {
					case SOBEL_FILTER_RESULT_ADDR:
						buffer.uint = o_result.read();
						break;
					default:
						std::cerr << "READ Error! SobelFilter::blocking_transport: address 0x" << std::setfill('0')
						          << std::setw(8) << std::hex << addr << std::dec << " is not valid" << std::endl;
				}
				data_ptr[0] = buffer.uc[0];
				data_ptr[1] = buffer.uc[1];
				data_ptr[2] = buffer.uc[2];
				data_ptr[3] = buffer.uc[3];
				break;
			case tlm::TLM_WRITE_COMMAND:
				// cout << "WRITE" << endl;
				switch (addr) {
					case SOBEL_FILTER_R_ADDR:
						i_r.write(data_ptr[0]);
						i_g.write(data_ptr[1]);
						i_b.write(data_ptr[2]);
						break;
					default:
						std::cerr << "WRITE Error! SobelFilter::blocking_transport: address 0x" << std::setfill('0')
						          << std::setw(8) << std::hex << addr << std::dec << " is not valid" << std::endl;
				}
				break;
			case tlm::TLM_IGNORE_COMMAND:
				payload.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
				return;
			default:
				payload.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
				return;
		}
		payload.set_response_status(tlm::TLM_OK_RESPONSE);  // Always OK
	}
};
#endif
