using System;
using System.Collections.Generic;
using System.Text;
using NAudio.Wave;
using System.Diagnostics;
using System.ComponentModel.Composition;

namespace AudioFileInspector
{
    [Export(typeof(IAudioFileInspector))]
    public class WaveFileInspector : IAudioFileInspector
    {
        #region IAudioFileInspector Members

        public string FileExtension
        {
            get { return ".wav"; }
        }

        public string FileTypeDescription
        {
            get { return "Wave File"; }
        }

        public string Describe(string fileName)
        {
            StringBuilder stringBuilder = new StringBuilder();
            using (WaveFileReader wf = new WaveFileReader(fileName))
            {
                stringBuilder.AppendFormat("{0} {1}Hz {2} channels {3} bits per sample\r\n", 
                    wf.WaveFormat.Encoding, wf.WaveFormat.SampleRate,
                    wf.WaveFormat.Channels, wf.WaveFormat.BitsPerSample);
                stringBuilder.AppendFormat("Extra Size: {0} Block Align: {1} Average Bytes Per Second: {2}\r\n",
                    wf.WaveFormat.ExtraSize, wf.WaveFormat.BlockAlign,
                    wf.WaveFormat.AverageBytesPerSecond);
                stringBuilder.AppendFormat("WaveFormat: {0}\r\n",wf.WaveFormat);
                
                stringBuilder.AppendFormat("Length: {0} bytes: {1} \r\n", wf.Length, wf.TotalTime);
                foreach (RiffChunk chunk in wf.ExtraChunks)
                {
                    stringBuilder.AppendFormat("Chunk: {0}, length {1}\r\n", chunk.IdentifierAsString, chunk.Length);
                    byte[] data = wf.GetChunkData(chunk);
                    DescribeChunk(chunk, stringBuilder, data);
                }
            }
            return stringBuilder.ToString();
        }

        private static void DescribeChunk(RiffChunk chunk, StringBuilder stringBuilder, byte[] data)
        {
            switch(chunk.IdentifierAsString)
            {
                case "strc":
                    DescribeStrc(stringBuilder, data);
                    break;
                default:
                    {
                        int n = 0;
                        foreach (byte b in data)
                        {
                            stringBuilder.AppendFormat("{0:X2} ", b);
                            if (++n % 8 == 0)
                                stringBuilder.Append("\r\n");
                        }
                        stringBuilder.Append("\r\n");
                    }
                    break;
            }
        }

        private static void DescribeStrc(StringBuilder stringBuilder, byte[] data)
        {
            // First 28 bytes are header
            int header1 = BitConverter.ToInt32(data, 0); // always 0x1C?
            int sliceCount = BitConverter.ToInt32(data, 4);
            int header2 = BitConverter.ToInt32(data, 8); // 0x19 or 0x41?
            int header3 = BitConverter.ToInt32(data, 12); // 0x05 or 0x0A? (linked with header 2 - 0x41 0x05 go together and 0x19 0x0A go together)
            int header4 = BitConverter.ToInt32(data, 16); // always 1?
            int header5 = BitConverter.ToInt32(data, 20); // 0x00, 0x01 or 0x0A?
            int header6 = BitConverter.ToInt32(data, 24); // 0x02, 0x04. 0x05

            stringBuilder.AppendFormat("{0} slices. unknown: {1},{2},{3},{4},{5},{6}\r\n",
                sliceCount,header1,header2,header3,header4,header5,header6);

            int offset = 28;

            for (int slice = 0; slice < sliceCount; slice++)
            {
                int unknown1 = BitConverter.ToInt32(data, offset); // 0 or 2
                int uniqueId1 = BitConverter.ToInt32(data, offset + 4); // another unique ID - doesn't change?

                long samplePosition = BitConverter.ToInt64(data, offset + 8);
                long samplePos2 = BitConverter.ToInt64(data, offset + 16); // is zero the first time through, equal to sample position next time round
                int unknown5 = BitConverter.ToInt32(data, offset + 24); // large number first time through, zero second time through, not flags, not a float
                int uniqueId2 = BitConverter.ToInt32(data, offset + 28); // always the same
                offset += 32;
                stringBuilder.AppendFormat("Pos: {2},{3} unknown: {0},{4}\r\n",
                    unknown1, uniqueId1, samplePosition, samplePos2, unknown5, uniqueId2);
            }
        }

        #endregion
    }
}
