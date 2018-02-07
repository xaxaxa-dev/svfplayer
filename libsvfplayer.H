#ifndef __LIBSVFPLAYER_H
#define __LIBSVFPLAYER_H

#include <stdint.h>
#include <limits.h>
#include <memory.h>
#include <string>
#include <vector>
#include <stdexcept>
using namespace std;


#define ARRSIZE(x) (sizeof((x))/sizeof(*(x)))
typedef unsigned char uchar;


//##########################################################################################
/***************** constants and stateless utility functions *****************/
//##########################################################################################
const char* svfStates[] = {"UNDEFINED","UNKNOWN","RESET","IDLE","DRSELECT",
	"DRCAPTURE","DRSHIFT","DREXIT1","DRPAUSE","DREXIT2","DRUPDATE",
	"IRSELECT","IRCAPTURE","IRSHIFT","IREXIT1","IRPAUSE","IREXIT2",
	"IRUPDATE"};
enum class svfState {
	UNDEFINED=0,
	UNKNOWN,
	RESET,IDLE,DRSELECT,
	DRCAPTURE,DRSHIFT,DREXIT1,DRPAUSE,DREXIT2,DRUPDATE,
	IRSELECT,IRCAPTURE,IRSHIFT,IREXIT1,IRPAUSE,IREXIT2,
	IRUPDATE
};
vector<svfState> svfAllStates={svfState::UNDEFINED};
#define svfIRStates svfState::IRSELECT, \
		svfState::IRCAPTURE, \
		svfState::IRSHIFT, \
		svfState::IREXIT1, \
		svfState::IRPAUSE, \
		svfState::IREXIT2, \
		svfState::IRUPDATE
#define svfDRStates svfState::DRSELECT, \
		svfState::DRCAPTURE, \
		svfState::DRSHIFT, \
		svfState::DREXIT1, \
		svfState::DRPAUSE, \
		svfState::DREXIT2, \
		svfState::DRUPDATE

//at each source state, the list of dst states that you should output '0' (on tms) on
vector<svfState> svfPathTable[]={
	{},											//UNDEFINED
	{},											//UNKNOWN
	svfAllStates,								//RESET
	{},											//IDLE
	{svfDRStates},								//DRSELECT
	{svfState::DRSHIFT},						//DRCAPTURE
	{},											//DRSHIFT
	{svfState::DRSHIFT,svfState::DRPAUSE,
		svfState::DREXIT2},						//DREXIT1
	{},											//DRPAUSE
	{svfState::DRSHIFT,svfState::DRPAUSE,
		svfState::DREXIT1},						//DREXIT2
	{svfState::IDLE},							//DRUPDATE
	
	{svfIRStates},								//IRSELECT
	{svfState::IRSHIFT},						//IRCAPTURE
	{},											//IRSHIFT
	{svfState::IRSHIFT,svfState::IRPAUSE,
		svfState::IREXIT2},						//IREXIT1
	{},											//IRPAUSE
	{svfState::IRSHIFT,svfState::IRPAUSE,
		svfState::IREXIT1},						//IREXIT2
	{svfState::IDLE}							//IRUPDATE
};
svfState svfTransitionTable[] = {
	//	0						1
	svfState::UNDEFINED,	svfState::UNDEFINED,	//UNDEFINED
	svfState::UNKNOWN,		svfState::UNKNOWN,		//UNKNOWN
	svfState::IDLE,			svfState::RESET,		//RESET	
	svfState::IDLE,			svfState::DRSELECT,		//IDLE
	svfState::DRCAPTURE,	svfState::IRSELECT,		//DRSELECT
	svfState::DRSHIFT,		svfState::DREXIT1,		//DRCAPTURE
	svfState::DRSHIFT,		svfState::DREXIT1,		//DRSHIFT
	svfState::DRPAUSE,		svfState::DRUPDATE,		//DREXIT1
	svfState::DRPAUSE,		svfState::DREXIT2,		//DRPAUSE
	svfState::DRSHIFT,		svfState::DRUPDATE,		//DREXIT2
	svfState::IDLE,			svfState::DRSELECT,		//DRUPDATE
	svfState::IRCAPTURE,	svfState::RESET,		//IRSELECT
	svfState::IRSHIFT,		svfState::IREXIT1,		//IRCAPTURE
	svfState::IRSHIFT,		svfState::IREXIT1,		//IRSHIFT
	svfState::IRPAUSE,		svfState::IRUPDATE,		//IREXIT1
	svfState::IRPAUSE,		svfState::IREXIT2,		//IRPAUSE
	svfState::IRSHIFT,		svfState::IRUPDATE,		//IREXIT2
	svfState::IDLE,			svfState::DRSELECT		//IRUPDATE
};

const char* svfOps[] = {"UNDEFINED","ENDDR","ENDIR","FREQUENCY",
	"HDR","HIR","RUNTEST","SDR","SIR","STATE","TDR","TIR","TRST"};
enum class svfOp {
	UNDEFINED=0,ENDDR,ENDIR,FREQUENCY,
	HDR,HIR,RUNTEST,SDR,SIR,STATE,TDR,TIR,TRST
};
svfState svfLookupState(const char* s) {
	int cnt=ARRSIZE(svfStates);
	for(int i=0;i<cnt;i++) {
		if(strcmp(s,svfStates[i])==0)
			return (svfState)i;
	}
	return svfState::UNDEFINED;
}
svfOp svfLookupOp(const char* s) {
	int cnt=ARRSIZE(svfOps);
	for(int i=0;i<cnt;i++) {
		if(strcmp(s,svfOps[i])==0)
			return (svfOp)i;
	}
	return svfOp::UNDEFINED;
}
uchar parseHexChar(char c) {
	uchar c2=(uchar)c;
	if(c2>=(uchar)'0' && c2<=(uchar)'9') return c2-(uchar)'0';
	if(c2>=(uchar)'a' && c2<=(uchar)'f') return c2-(uchar)'a'+10;
	if(c2>=(uchar)'A' && c2<=(uchar)'F') return c2-(uchar)'A'+10;
	return 255;
}
//returns empty string on error
string svfParseHex(const char* s, int len) {
	string out;
	bool incomplete=false;
	for(const char* ch=s+len-1;ch>=s;ch--) {
		uchar halfByte=parseHexChar(*ch);
		if(halfByte==255) return string();
		if(incomplete) {
			out[out.length()-1]=((uchar)out[out.length()-1])|(halfByte<<4);
		} else {
			out+=halfByte;
		}
		incomplete=!incomplete;
	}
	return out;
}

//##########################################################################################
/***************** parser *****************/
//##########################################################################################
struct svfData {
	//data is in raw form, with the LSB in the lowest numbered bit (i.e. data[0]&0x1)
	string tdiData,tdoData,tdiMask,tdoMask;
	int dataLen;
	svfData(): dataLen(0) {}
};
struct svfCommand {
	svfOp op;
	//in the case of RUNTEST, data.dataLen specifies the number of
	//clock rising edges while in RUN-TEST/IDLE state
	svfData data;
	double frequency;
	vector<svfState> states;
};
struct svfParser {
	//usage: call reset(), then read one line at a time from the svf file;
	//each time a line is read,
	//1. call processLine() and pass in the line;
	//		the parser will retain a reference to the line; do not free it yet
	//2. repeatedly call nextCommand() until false is returned
	//3. free the line
	
	int lineNum=0;
	const char* curLine=NULL;
	int curLineLen=0;
	string buf;
	int bufI;
	void reset() {
		lineNum=0;
		curLine=NULL;
		curLineLen=0;
		buf.clear();
	}
	void processLine(const char* line, int len) {
		lineNum++;
		if(len>=2 && line[0]=='/' && line[1]=='/') return;
		curLine=line;
		curLineLen=len;
	}
	bool nextCommand(svfCommand& out) {
		if(!_readCommand()) return false;
		//command text is in buf
		_beginRead();
		string cmd=_readWord();
		out.op=svfLookupOp(cmd.c_str());
		switch(out.op) {
		case svfOp::UNDEFINED:
			_parseError("unknown svf command: "+cmd);
			break;
		case svfOp::ENDDR:
		case svfOp::ENDIR:
		{
			string st=_readWord();
			out.states.clear();
			out.states.push_back(svfLookupState(st.c_str()));
			if(out.states[0]==svfState::UNDEFINED)
				_parseError("unknown state: "+st);
			break;
		}
		case svfOp::FREQUENCY:
			out.frequency=_readDouble();
			_expect("HZ");
			break;
		case svfOp::RUNTEST:
		{
			string st=_readWord(true);
			out.states.clear();
			out.states.push_back(svfLookupState(st.c_str()));
			if(out.states[0]!=svfState::UNDEFINED)
				_readWord();
			out.data.dataLen=_readInt();
			_expect("TCK");
			break;
		}
		case svfOp::STATE:
		{
			out.states.clear();
			string tmp;
			while((tmp=_readWord()).length()>0) {
				svfState st=svfLookupState(tmp.c_str());
				if(st==svfState::UNDEFINED)
					_parseError("unknown state: "+tmp);
				out.states.push_back(st);
			}
			break;
		}
		case svfOp::SDR:
		case svfOp::SIR:
		case svfOp::HDR:
		case svfOp::HIR:
		case svfOp::TDR:
		case svfOp::TIR:
		{
			out.data.dataLen=_readInt();
			out.data.tdiData.clear();
			out.data.tdoData.clear();
			out.data.tdiMask.clear();
			out.data.tdoMask.clear();
			string tmp;
			while((tmp=_readWord()).length()>0) {
				if(tmp.compare("TDI")==0) {
					out.data.tdiData=_readHexValue();
				} else if(tmp.compare("TDO")==0) {
					out.data.tdoData=_readHexValue();
				} else if(tmp.compare("MASK")==0) {
					out.data.tdoMask=_readHexValue();
				} else if(tmp.compare("SMASK")==0) {
					out.data.tdiMask=_readHexValue();
				} else {
					_parseError("unknown attribute: "+tmp);
				}
			}
			// if TDO was specified but not MASK, assume a mask of all 1s
			/*if(out.data.tdoMask == "" && out.data.tdoData != "") {
				out.data.tdoMask = string(out.data.tdoData.length(), 0xff);
				assert(out.data.tdoMask.length() == out.data.tdoData.length());
			}*/
			break;
		}
		case svfOp::TRST:
			_expect("OFF");
			break;
		default:
			_parseError("unknown svf command: "+cmd);
			break;
		}
		_skipSpaces();
		if(bufI<(int)buf.length()) {
			fprintf(stderr,"%d %d %d\n",bufI,(int)buf.length(),(int)buf[bufI]);
			_parseError("garbage after command: "+buf.substr(bufI));
		}
		buf.clear();
		return true;
	}
	int _findChr(const char* s, int len, char c) {
		const void* tmp=memchr(s,c,len);
		if(tmp==NULL) return -1;
		return ((char*)tmp)-s;
	}
	
	//reads the next command text and stores it in buf
	bool _readCommand() {
		if(curLine==NULL || curLineLen==0) return false;
		int semiColon=_findChr(curLine,curLineLen,';');
		if(semiColon<0) {
			//end of current command not found; append rest of the data to buffer
			buf.append(curLine,curLineLen);
			curLine=NULL;
			curLineLen=0;
			return false;
		}
		buf.append(curLine,semiColon);
		curLine+=semiColon+1;
		curLineLen-=(semiColon+1);
		if(curLineLen==0) curLine=NULL;
		return true;
	}
	
	//cmd buffer manipulation functions
	void _beginRead() {
		bufI=0;
	}
	void _skipSpaces() {
		while(bufI<(int)buf.length() && isspace(buf[bufI])) bufI++;
	}
	string _readWord(bool peek=false) {
		_skipSpaces();
		int i=bufI;
		while(i<(int)buf.length() && !isspace(buf[i])) i++;
		string s=buf.substr(bufI,i-bufI);
		if(!peek) bufI=i;
		return s;
	}
	string _readHexValue() {
		_expectChar('(');
		string s=_readWord();
		while(s.length()>0 && s[s.length()-1]!=')') {
			string tmp=_readWord();
			if(tmp.length()<=0) break;
			s+=tmp;
		}
		if(s[s.length()-1]!=')') {
			_expectChar(')');
			return svfParseHex(s.data(),s.length());
		}
		return svfParseHex(s.data(),s.length()-1);
	}
	int _readInt() {
		string s=_readWord();
		if(s.length()==0) _parseError("expected integer");
		const char* nptr=s.c_str();
		char* endptr=NULL;
		long tmp=strtol(nptr,&endptr,10);
		if(endptr==nptr) _parseError("expected integer");
		if(tmp>INT_MAX || tmp<INT_MIN) _parseError("integer overflow");
		return (int)tmp;
	}
	double _readDouble() {
		string s=_readWord();
		if(s.length()==0) _parseError("expected number");
		const char* nptr=s.c_str();
		char* endptr=NULL;
		double tmp=strtod(nptr,&endptr);
		if(endptr==nptr) _parseError("expected number");
		return tmp;
	}
	void _expect(const char* x) {
		string s=_readWord();
		if(s.compare(x)!=0) _parseError("expecting: "+string(x));
	}
	void _expectChar(char c) {
		_skipSpaces();
		if(bufI>=(int)buf.length() || buf[bufI]!=c) _parseError(string("expecting: ")+c);
		bufI++;
	}
	//misc
	void _parseError(string desc) {
		char buf[128];
		snprintf(buf,128,"line %d: ",lineNum);
		throw runtime_error(string(buf)+desc);
	}
};

//##########################################################################################
/***************** player *****************/
//##########################################################################################
struct svfPlayer {
	svfState endDR,endIR,runTestState;
	svfState deviceState;
	svfData headerIR,headerDR,trailerIR,trailerDR,defaultIR,defaultDR;
	
	//one byte per clock cycle; format of each byte:
	//	bit 0: value to put on tms
	//	bit 1: value to put on tdi
	//	bit 2: value expected on tdo
	//	bit 3: 0 if tdi is don't care, 1 otherwise
	//	bit 4: 0 if tdo is don't care, 1 otherwise
	string outBuffer;
	
	void reset() {
		endDR=endIR=runTestState=svfState::IDLE;
		deviceState=svfState::UNKNOWN;
	}
	void processCommand(const svfCommand& cmd) {
		switch(cmd.op) {
			case svfOp::ENDDR:
				endDR=cmd.states[0];
				break;
			case svfOp::ENDIR:
				endIR=cmd.states[0];
				break;
			case svfOp::FREQUENCY:
				_warn("FREQUENCY command not implemented. ignoring.");
				break;
			case svfOp::HDR:
			case svfOp::HIR:
			case svfOp::TDR:
			case svfOp::TIR:
			{
				svfData& dst=(cmd.op==svfOp::HDR)?headerDR:(
							(cmd.op==svfOp::HIR)?headerIR:(
							(cmd.op==svfOp::TDR)?trailerDR:(
							trailerIR)));
				if(cmd.data.dataLen!=dst.dataLen) {
					dst=cmd.data;
					padData(dst);
				} else {
					if(cmd.data.tdiData.length()!=0)
						dst.tdiData=cmd.data.tdiData;
					if(cmd.data.tdoData.length()!=0)
						dst.tdoData=cmd.data.tdoData;
					if(cmd.data.tdiMask.length()!=0)
						dst.tdiMask=cmd.data.tdiMask;
					if(cmd.data.tdoMask.length()!=0)
						dst.tdoMask=cmd.data.tdoMask;
				}
				break;
			}
			case svfOp::RUNTEST:
			{
				svfState st=cmd.states[0];
				if(st==svfState::UNDEFINED)
					st=runTestState;
				else runTestState=st;
				doRunTest(st,cmd.data.dataLen);
				break;
			}
			case svfOp::SDR:
			case svfOp::SIR:
			{
				if(cmd.data.dataLen<=0) _err("length must be greater than zero");
				bool ir=(cmd.op==svfOp::SIR);
				svfData& header=ir?headerIR:headerDR;
				svfData& trailer=ir?trailerIR:trailerDR;
				svfData& old=ir?defaultIR:defaultDR;
				if(old.dataLen!=cmd.data.dataLen) {
					old=cmd.data;
					padData(old);
				} else {
					if(cmd.data.tdiData.length()!=0)
						old.tdiData=cmd.data.tdiData;
					old.tdoData=cmd.data.tdoData;
					if(cmd.data.tdiMask.length()!=0)
						old.tdiMask=cmd.data.tdiMask;
					if(cmd.data.tdoMask.length()!=0)
						old.tdoMask=cmd.data.tdoMask;
					padData(old);
				}
				
				goToState(ir?svfState::IRSHIFT:svfState::DRSHIFT);
				doShift(header);
				doShift(old);
				doShift(trailer);
				outBuffer[outBuffer.length()-1]|=1;
				calculateTransition(1);
				goToState(ir?endIR:endDR);
				break;
			}
			case svfOp::STATE:
				for(int i=0;i<(int)cmd.states.size();i++) {
					goToState(cmd.states[i]);
				}
				break;
			default:
				_warn("command not implemented: "+string(svfOps[(int)cmd.op]));
				break;
		}
	}
	void padData(svfData& data) {
		if(data.dataLen==0) return;
		int bytes=(data.dataLen+7)/8;
		if(data.tdiData.length()==0) _err("TDI data required");
		if(data.tdiMask.length()==0) data.tdiMask.assign(bytes,255);
		if(data.tdoData.length()==0) {
			data.tdoData.assign(bytes,0);
			data.tdoMask.assign(bytes,0);
			return;
		}
		if(data.tdoMask.length()==0) data.tdoMask.assign(bytes,255);
	}
	void doShift(const svfData& data, bool exit=false) {
		for(int i=0;i<data.dataLen;i++) {
			bool tms,tdi,tdo,tdiEnable,tdoEnable;
			int mask=1<<(i%8);
			tms=((i==(data.dataLen-1)) && exit);
			tdi=(int(data.tdiData[i/8])&mask)!=0;
			tdo=(int(data.tdoData[i/8])&mask)!=0;
			tdiEnable=(int(data.tdiMask[i/8])&mask)!=0;
			tdoEnable=(int(data.tdoMask[i/8])&mask)!=0;
			outBuffer+=char(tms|(tdi<<1)|(tdo<<2)|(tdiEnable<<3)|(tdoEnable<<4));
		}
	}
	void doRunTest(svfState st, int count) {
		goToState(st);
		int tms=(st==svfState::RESET)?1:0;
		for(int i=0;i<count;i++)
			doTransition(tms);
	}
	void goToState(svfState st) {
	_begin:
		if(deviceState==st) return;
		if(deviceState==svfState::UNKNOWN) {
			doTransition(1); doTransition(1); doTransition(1);
			doTransition(1); doTransition(1); doTransition(1);
			deviceState=svfState::RESET;
			goto _begin;
		}
		const vector<svfState>& table=svfPathTable[(int)deviceState];
		for(int i=0;i<(int)table.size();i++) {
			if(table[i]==svfState::UNDEFINED ||		//all match
				table[i]==st) {
				doTransition(0);
				calculateTransition(0);
				goto _begin;
			}
		}
		doTransition(1);
		calculateTransition(1);
		goto _begin;
	}
	
	inline void calculateTransition(int tms) {
		deviceState=svfTransitionTable[int(deviceState)*2+tms];
	}
	
	inline void doTransition(int tms) {
		outBuffer+=char(tms);		//all other bit fields are zero
	}
	void _warn(string msg) {
		fprintf(stderr,"warning: %s\n",msg.c_str());
	}
	void _err(string msg) {
		throw runtime_error("error: "+msg);
	}
};

#endif

