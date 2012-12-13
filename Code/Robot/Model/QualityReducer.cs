using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace HumanoidRobot.Model
{
    public class QualityReducer
    {
        public static byte[] Reduce(byte[] buffer)
        {
            byte[] reducedBuffer = new byte[buffer.Length / 4];

            for (int i = 0; i < 240; i++)
            {
                for (int j = 0; j < 1280; j += 4)
                {
                    reducedBuffer[i * 1280 + j] = buffer[2 * i * 2560 + 2 * j];
                    reducedBuffer[i * 1280 + j + 1] = buffer[2 * i * 2560 + 2 * j + 1];
                    reducedBuffer[i * 1280 + j + 2] = buffer[2 * i * 2560 + 2 * j + 2];
                }
            }
                return reducedBuffer;
        }

            public static byte[] Reduce2(byte[] buffer)
        {
            byte[] reducedBuffer = new byte[buffer.Length/4];

            for (int i = 0; i < 120; i++)
			{
                for (int j = 0; j < 640; j+=4)
			    {
                    reducedBuffer[i * 640 + j] = buffer[2 * i * 1280 + 2 * j];
                    reducedBuffer[i * 640 + j + 1] = buffer[2 * i * 1280 + 2 * j + 1];
                    reducedBuffer[i * 640 + j + 2] = buffer[2 * i * 1280 + 2 * j + 2];
			    }
			}
                 return reducedBuffer;
        }
    }
}
