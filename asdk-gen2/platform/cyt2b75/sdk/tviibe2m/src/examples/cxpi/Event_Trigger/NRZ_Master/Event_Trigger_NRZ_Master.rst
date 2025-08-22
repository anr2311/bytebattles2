CXPI: Event Trigger – Master – NRZ Mode 
=======================================
Description
^^^^^^^^^^^
- This example demonstrates CXPI transmission in Event Trigger method.
- TVII acts as a master node of CXPI network.

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B9x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board
- CYTVII-B-E-BB Rev.A Board

Dependency
^^^^^^^^^^
- CYTVII-B-E-1M-176-CPU board should be connected on CYTVII-B-E-BB board.
- Open J11, J12, J14 on the base board. Short J6, J9, J13 on the base board.
- Connect following pints on the base board
   - (RX) JP5.9 to J14.1, (TX) JP5.7 to J11.1, (CLK) JP5.6 to J12.2
- Connect CXPI connector (P1) on the base board and CXPI analyzer if available.

Expectation
^^^^^^^^^^^
- Scheduled frames: TVII send PIDs according to below schedule
  +-----+-----+-----------------+----------------------+----------------------+----------------+
  | No. | ID  | PID transmitter | Response transmitter | Frame length (bytes) | Frame interval |
  +=====+=====+=================+======================+======================+================+
  | 1   | 0A  | T2G             | CXPI analyzer        | 8                    | 100 ms         |
  +-----+-----+-----------------+----------------------+----------------------+----------------+
  | 2   | 4A  | T2G             | T2G                  | 8                    | 100 ms         |
  +-----+-----+-----------------+----------------------+----------------------+----------------+
  | 3   | 0F  | T2G             | CXPI analyzer        | 16                   | 100 ms         |
  +-----+-----+-----------------+----------------------+----------------------+----------------+
  | 4   | 4F  | T2G             | T2G                  | 16                   | 100 ms         |
  +-----+-----+-----------------+----------------------+----------------------+----------------+
  TVII copy the response and send back to tester, 1->2, and 3->4
- Event frames: Press SW1, TVII send PID = 0x4D and response to tester
  +-----+-----+-----------------+----------------------+----------------------+
  | No. | ID  | PID transmitter | Response transmitter | Frame length (bytes) |
  +=====+=====+=================+======================+======================+
  | 5   | 4D  | T2G             | T2G                  | 8                    |
  +-----+-----+-----------------+----------------------+----------------------+
- CXPI analyzer
   - If you have PX-10 CXPI analyzer, you can use cxpi_test.mps for it.