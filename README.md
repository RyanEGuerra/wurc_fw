# wurc_fw

TI Code Composer Studio firmware project for the WURC SDR platform containing full custom driver library for the LMS6002D and running on the Stellaris LM3S5R36.

The project provides for a number of features centering around the Stellaris LM3S5R36 from TI:
* a complete calibration, configuration, and control library for the LMS6002D built from scratch.
* a robust USB serial command line interface supplying access to configuration macros for the system transceiver.
* interrupt-based Tx/Rx fast switching with highly-optimized assembly implemention.

This code was initially developed for the Skylark Wireless Wideband UHF Radio Card (WURC), a Software-Defined Radio (SDR) analog front-end transceiver for research & development. More information can be found at: www.skylarkwireless.com/WURC

Author: Ryan E. Guerra (ryan@guerra.rocks)
Author: Dr. Narendra Anand
Thanks: Holly Liang

# Texas Instruments Legal Disclaimer
THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
DAMAGES, FOR ANY REASON WHATSOEVER.

# Personal Legal Disclaimer & License
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.

(c) Ryan E. Guerra ryan@guerra.rocks 2012-2016

# License
The code contained in this repository is licensed under the Apache 2.0 Software License.
http://www.apache.org/licenses/LICENSE-2.0

