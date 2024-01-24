// Copyright 2019 Fraunhofer IPA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LEUZE_UDPSIM_H
#define LEUZE_UDPSIM_H

#include <iostream>
#include <boost/bind/bind.hpp>
#include <boost/thread.hpp>


class UdpSim
{
public:

    static void data_generator(boost::function<void(std::basic_string<unsigned char>)> handle_read)
    ;
#if 0
    {
        //This is a simulation => Generate dummy frames
        std::basic_string<unsigned char> test_buffer;

        uint32_t scan_number = 0;

        while (true)
        {

#if defined (RSL400)

            //State image (Frame 0 - first frame)
            test_buffer.resize(48);
            test_buffer = {
                        0x30, 0x00,                            //Header H1 (Frame size)
                        0x00, 0x00, 0x08, 0x00, 0xaf, 0xfe,    //Header H1 (rest)
                        0x04, 0x15, 0x02, 0xc8,                //Header H2
                        0x01, 0x00,                            //ID = 1 (Extended status profile)
                        0x00, 0x00,                            //Block number
                        static_cast<unsigned char>(scan_number & 0xFF),                                     //Scan number 0. byte
                        static_cast<unsigned char>(static_cast<unsigned char>(scan_number >> 8)  & 0xFF),   //Scan number 1. byte
                        static_cast<unsigned char>(static_cast<unsigned char>(scan_number >> 16) & 0xFF),   //Scan number 2. byte
                        static_cast<unsigned char>(static_cast<unsigned char>(scan_number >> 24) & 0xFF),   //Scan number 3. byte
                        0x01, 0x01, 0x02, 0x80, 0x00, 0x00, 0x10, 0x80, 0x6e, 0x29, 0x05, 0x00, 0xe8, 0x11, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, //Status profile
                        0x00, 0x00,  // Measurement Contour Description (start index)
                        0xa8, 0x0b,     // Measurement Contour Description (stop index)
                        0x01, 0x00,  // Measurement Contour Description (beam step)
                        0x00, 0x00    // Reserved
                        };

            handle_read(test_buffer);

            //Meas data (Frames 1 to 8)
            for (unsigned char meas_frame_counter = 0; meas_frame_counter < 8; meas_frame_counter++)
            {
                test_buffer.clear();

                //Calculate number of beams in this frame:
                uint16_t beam_cout = 720; //Maximum number of beams in a full frame
                if (meas_frame_counter == 7)
                {
                    //This is the last beam which contains fewer beams:
                    beam_cout = 360;
                }

                //Calculate total frame size
                uint16_t frame_size = beam_cout*2 + 20; //We must multiply the beams by 2 because each beam takes up 2 bytes; 20 ist the size of the header, example: 1460 for a full frame

 test_buffer.resize(frame_size); 
                //Fill header
                test_buffer = {
                            static_cast<unsigned char>(frame_size & 0xFF), static_cast<unsigned char>(frame_size >> 8) ,     //H1 (Frame size)
                            0x00 ,0x00 ,0x08 ,0x00 ,0xaf ,0xfe ,  //Header H1 (rest)
                            0x04 ,0x15 ,0x02 ,0xc8 ,              //Header H2
                            0x03 , 0x00,                          //ID = 3 (distances and amplitudes)
                            meas_frame_counter ,0x00,             //Block number
                            static_cast<unsigned char>(scan_number & 0xFF),                                     //Scan number 0. byte
                            static_cast<unsigned char>(static_cast<unsigned char>(scan_number >> 8)  & 0xFF),   //Scan number 1. byte
                            static_cast<unsigned char>(static_cast<unsigned char>(scan_number >> 16) & 0xFF),   //Scan number 2. byte
                            static_cast<unsigned char>(static_cast<unsigned char>(scan_number >> 24) & 0xFF),   //Scan number 3. byte
                        };

                //test_buffer.resize(frame_size); 

                //cast the buffer from 8-bit array into a 16-bit array, skip the header:
                uint16_t* test_buffer_as_shorts = reinterpret_cast<uint16_t*>(&test_buffer[20]);

                //Fill meas data with a simulation pattern:
                for (int i = 0; i < beam_cout; i++)
                {
                    test_buffer_as_shorts[i] = 1000 + static_cast<int16_t>(100*sin(i + scan_number));
                }            
                handle_read(test_buffer);
            }
            
            sleep(0.040);       //wait 40ms (real cycle time of RSL400)


#else
    #error "No scanner type defined! Cannot create simulation!"
#endif

            //Increase the scan number:
            scan_number++;
        }
    }
#endif
};

#endif //LEUZE_UDPSIM_H
