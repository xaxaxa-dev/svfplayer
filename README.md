libsvfplayer
================
libsvfplayer is a JTAG .svf parser and player library in C++, designed to be generic and easy to use, consisting of these classes:
* svfParser - lexer and parser for the SVF language
* svfPlayer - takes svf commands (can be from a svfParser) and generates a JTAG waveform that can be replayed (e.g. by bit-banging or dma gpio)

Example applications:
* Programming an FPGA without a xilinx or altera programmer (using e.g. a orange pi or any SBC with gpio)
* Automated programmers for production use

Example usage:
```c++
svfParser parser;
svfPlayer player;

parser.reset();
player.reset();
while(true) {
  char* line=READ A LINE FROM SVF FILE;
  parser.processLine(line,strlen(line));
  svfCommand cmd;
  while(parser.nextCommand(cmd))
    player.processCommand(cmd);
  
  // player.outBuffer now contains a fragment of a JTAG waveform; you can store it or play it on your hardware
  // outBuffer contains one byte per clock cycle; format of each byte (bit 0 means LSB):
  //	bit 0: value to put on tms
  //	bit 1: value to put on tdi
  //	bit 2: value expected on tdo
  //	bit 3: 0 if tdi is don't care, 1 otherwise
  //	bit 4: 0 if tdo is don't care, 1 otherwise
  
  free(line);
}
  
```

svfplayer.C contains an example command line tool to play .svf files on some in-house custom hardware. It isn't useful as a tool but can be a reference for how to use the libsvfplayer API.
