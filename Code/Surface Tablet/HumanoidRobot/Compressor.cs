using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using System.Text;


namespace HumanoidRobot
{
    public class Compressor
    {
        //public static byte[] Compress(byte[] buffer)
        //{
        //    MemoryStream ms = new MemoryStream();
        //    GZipStream zip = new GZipStream(ms, CompressionMode.Compress, true);
        //    zip.Write(buffer, 0, buffer.Length);

        //    ms.Position = 0;

        //    MemoryStream outStream = new MemoryStream();
        //    byte[] compressed = new byte[ms.Length];
        //    ms.Read(compressed)
        //}

        public static byte[] Decompress(byte[] gzBuffer)
        {
            MemoryStream ms = new MemoryStream();
            int msgLength = BitConverter.ToInt32(gzBuffer, 0);
            ms.Write(gzBuffer, 4, gzBuffer.Length - 4);

            byte[] buffer = new byte[msgLength];

            ms.Position = 0;
            GZipStream zip = new GZipStream(ms, CompressionMode.Decompress);
            zip.Read(buffer, 0, buffer.Length);
            return buffer;
        }
    }
}
