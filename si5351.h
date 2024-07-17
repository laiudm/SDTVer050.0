// inspired by uSDX code at https://github.com/threeme3/usdx

#define F_XTAL ((uint32_t) (25000000.0 * 5.000600 / 5.0) )
#define SI5351_ADDR 0x60
#define SI_CLK_OE 3

// shims to work with T41 code
//#define SI5351_FREQ_MULT 100UL
#define SI5351_FREQ_MULT 1UL
enum si5351_clock {SI5351_CLK0, SI5351_CLK1, SI5351_CLK2, SI5351_CLK3,
  SI5351_CLK4, SI5351_CLK5, SI5351_CLK6, SI5351_CLK7};
enum si5351_pll {SI5351_PLLA, SI5351_PLLB};  
enum si5351_drive {SI5351_DRIVE_2MA, SI5351_DRIVE_4MA, SI5351_DRIVE_6MA, SI5351_DRIVE_8MA};
#define SI5351_CRYSTAL_LOAD_10PF        (3<<6)

class Si5351 {
public:
  int8_t i2c_bus_addr = SI5351_ADDR;
  int32_t _fout;
  uint8_t _div;  // note: uint8_t asserts fout > 3.5MHz with R_DIV=1
  uint16_t _msa128min512;
  uint32_t _msb128;
  uint8_t pll_regs[8];
  int16_t iqmsa = 0; // to detect a need for a PLL reset. External code sets this when needed - ugly.
  uint32_t fxtal = F_XTAL;

  #define BB0(x) ((uint8_t)(x))           // Bash byte x of int32_t
  #define BB1(x) ((uint8_t)((x)>>8))
  #define BB2(x) ((uint8_t)((x)>>16))

  #define FAST __attribute__((optimize("Ofast")))

  uint8_t SendRegister(uint8_t addr, uint8_t data) {
    // Serial.print("Send register "); Serial.print(addr); Serial.print(" "); Serial.println(data, 16);
    
    Wire.beginTransmission(i2c_bus_addr);
    Wire.write(addr);
    Wire.write(data);
    return Wire.endTransmission();
  }
  
  uint8_t SendRegisters(uint8_t addr, uint8_t *data, uint8_t bytes) {
    Wire.beginTransmission(i2c_bus_addr);
    Wire.write(addr);
    for(int i = 0; i < bytes; i++) {
      Wire.write(data[i]);
    }
    return Wire.endTransmission();
  }

  uint8_t RecvRegister(uint8_t addr) {
    uint8_t reg_val = 0;

    Wire.beginTransmission(i2c_bus_addr);
    Wire.write(addr);
    Wire.endTransmission();
    //Wire.requestFrom(i2c_bus_addr, (uint8_t)1, (uint8_t)false);
    Wire.requestFrom(i2c_bus_addr, 1, (int) false);
    while(Wire.available()) {
      reg_val = Wire.read();
    }
    return reg_val;
  }

  inline void SendPLLRegisterBulk(){
    SendRegisters(26+0*8 + 4, &pll_regs[4], 4);
  }
    /*
  void SendRegisters(uint8_t reg, uint8_t* data, uint8_t n){ // usdx - rewrite
    write_bulk(reg, n, data);
  } */

  inline void FAST freq_calc_fast(int16_t df) { // note: relies on cached variables: _msb128, _msa128min512, _div, _fout, fxtal
    #define _MSC  0x10000
    uint32_t msb128 = _msb128 + ((int64_t)(_div * (int32_t)df) * _MSC * 128) / fxtal;

    uint16_t msp1 = _msa128min512 + msb128 / _MSC; // = 128 * _msa + msb128 / _MSC - 512;
    uint16_t msp2 = msb128; // = msb128 % _MSC;  assuming MSC is covering exact uint16_t so the mod operation can dissapear (and the upper BB2 byte) // = msb128 - msb128/_MSC * _MSC;

    //Serial.print(" msp1: "); Serial.print(msp1); Serial.print(" msp2: "); Serial.print(msp2);Serial.print(" reg[5] = "); Serial.println(((_MSC&0xF0000)>>(16-4)));
    
    //pll_regs[0] = BB1(msc);  // 4 regs are constant
    //pll_regs[1] = BB0(msc);
    //pll_regs[2] = BB2(msp1);
    //pll_regs[3] = BB1(msp1);
    pll_regs[4] = BB0(msp1);
    pll_regs[5] = ((_MSC&0xF0000)>>(16-4))/*|BB2(msp2)*/; // top nibble MUST be same as top nibble of _MSC !  assuming that BB2(msp2) is always 0 -> so reg is constant
    pll_regs[6] = BB1(msp2);
    pll_regs[7] = BB0(msp2);
  }
  
  enum ms_t { PLLA=0, PLLB=1, MSNA=-2, MSNB=-1, MS0=0, MS1=1, MS2=2, MS3=3, MS4=4, MS5=5 };
  
  void ms(int8_t n, uint32_t div_nom, uint32_t div_denom, uint8_t pll = PLLA, uint8_t _int = 0, uint16_t phase = 0, uint8_t rdiv = 0){
    uint16_t msa; uint32_t msb, msc, msp1, msp2, msp3;
    msa = div_nom / div_denom;     // integer part: msa must be in range 15..90 for PLL, 8+1/1048575..900 for MS
    if(msa == 4) _int = 1;  // To satisfy the MSx_INT=1 requirement of AN619, section 4.1.3 which basically says that for MS divider a value of 4 and integer mode must be used
    msb = (_int) ? 0 : (((uint64_t)(div_nom % div_denom)*_MSC) / div_denom); // fractional part
    msc = (_int) ? 1 : _MSC;

    msp1 = 128*msa + 128*msb/msc - 512;
    msp2 = 128*msb - 128*msb/msc * msc;
    msp3 = msc;

    //Serial.print("n = "); Serial.print(n); Serial.print(" div_nom = "); Serial.print(div_nom); Serial.print(" div_denom = "); Serial.print(div_denom); 
    //Serial.print(" msa = "); Serial.print(msa); Serial.print(" msb = "); Serial.print(msb); Serial.print(" msc = "); Serial.print(msc); 
    //Serial.print(" msp1 = "); Serial.print(msp1); Serial.print(" msp2 = "); Serial.print(msp2); Serial.print(" reg[5] = "); Serial.println(BB2(((msp3 & 0x0F0000)<<4) | msp2));
        
    uint8_t ms_reg2 = BB2(msp1) | (rdiv<<4) | ((msa == 4)*0x0C);
    uint8_t ms_regs[8] = { BB1(msp3), BB0(msp3), ms_reg2, BB1(msp1), BB0(msp1), BB2(((msp3 & 0x0F0000)<<4) | msp2), BB1(msp2), BB0(msp2) };

    SendRegisters(n*8+42, ms_regs, 8); // Write to MSx
    if(n < 0){
      SendRegister(n+16+8, 0x80|(0x40*_int)); // MSNx PLLn: 0x40=FBA_INT; 0x80=CLKn_PDN
    } else {
      SendRegister(n+16, ((pll)*0x20)|0x0C|0|(0x40*_int));  // MSx CLKn: 0x0C=PLLA,0x2C=PLLB local msynth; 0=2mA; 0x40=MSx_INT; 0x80=CLKx_PDN
      //SendRegister(n+16, ((pll)*0x20)|0x0C|3|(0x40*_int));  // MSx CLKn: 0x0C=PLLA,0x2C=PLLB local msynth; 3=8mA; 0x40=MSx_INT; 0x80=CLKx_PDN
      SendRegister(n+165, (!_int) * phase * msa / 90);      // when using: make sure to configure MS in fractional-mode, perform reset afterwards
    }
  }

  void phase(int8_t n, uint32_t div_nom, uint32_t div_denom, uint16_t phase) {
    SendRegister(n+165, phase * (div_nom / div_denom) / 90);
  }  // when using: make sure to configure MS in fractional-mode!, perform reset afterwards

  void _reset(){ SendRegister(177, 0xA0); } // 0x20 reset PLLA; 0x80 reset PLLB

  void oe(uint8_t mask){ SendRegister(3, ~mask); } // output-enable mask: CLK2=4; CLK1=2; CLK0=1

  void freq(int32_t fout, uint16_t i, uint16_t q){  // Set a CLK0,1,2 to fout Hz with phase i, q (on PLLA)
      uint8_t rdiv = 0; // CLK pin sees fout/(2^rdiv)
      if(fout > 300000000){ i/=3; q/=3; fout/=3; }  // for higher freqs, use 3rd harmonic
      if(fout < 500000){ rdiv = 7; fout *= 128; } // Divide by 128 for fout 4..500kHz
      uint16_t d; if(fout < 30000000) d = (16 * fxtal) / fout; else d = (32 * fxtal) / fout;  // Integer part  .. maybe 44?
      if(fout < 3500000) d = (7 * fxtal) / fout;  // PLL at 189MHz to cover 160m (freq>1.48MHz) when using 27MHz crystal
      if(fout > 140000000) d = 4; // for f=140..300MHz; AN619; 4.1.3, this implies integer mode
      if(d % 2) d++;  // even numbers preferred for divider (AN619 p.4 and p.6)
      if( (d * (fout - 5000) / fxtal) != (d * (fout + 5000) / fxtal) ) d += 2; // Test if multiplier remains same for freq deviation +/- 5kHz, if not use different divider to make same
      uint32_t fvcoa = d * fout;  // Variable PLLA VCO frequency at integer multiple of fout at around 27MHz*16 = 432MHz
      // si5351 spectral purity considerations: https://groups.io/g/QRPLabs/message/42662

      //Serial.print(" fout = "); Serial.print(fout); Serial.print(" d = "); Serial.print(d); Serial.print(" fvcoa = "); Serial.println(fvcoa);

      ms(MSNA, fvcoa, fxtal);                   // PLLA in fractional mode
      //ms(MSNB, fvcoa, fxtal);
      ms(MS0,  fvcoa, fout, PLLA, 0, i, rdiv);  // Multisynth stage with integer divider but in frac mode due to phase setting
      ms(MS1,  fvcoa, fout, PLLA, 0, q, rdiv);
      ms(MS2,  fvcoa, fout, PLLA, 0, 0, rdiv);  // CLK2 output

      if(iqmsa != (((int8_t)i-(int8_t)q)*((int16_t)(fvcoa/fout))/90)){ iqmsa = ((int8_t)i-(int8_t)q)*((int16_t)(
        fvcoa/fout))/90; reset();
       }
      //oe(0b0000011);  // output enable CLK0, CLK1
      oe(0b0000111);  // output enable CLK0, CLK1, CLK2

// #define x // x isn't defined. This code has been tested. Simply uncommenting it doesn't work. Not surprising because it duplicates some of the above code. More work required.
#define F_MCU 16000000
#ifdef x  
      ms(MSNA, fvcoa, fxtal);
      ms(MSNB, fvcoa, fxtal);
      #define F_DEV 4
      ms(MS0,  fvcoa, (fout + F_DEV), PLLA, 0, 0, rdiv);
      ms(MS1,  fvcoa, (fout + F_DEV), PLLA, 0, 0, rdiv);
      ms(MS2,  fvcoa, fout, PLLA, 0, 0, rdiv);
      reset();
      ms(MS0,  fvcoa, fout, PLLA, 0, 0, rdiv);
      delayMicroseconds(F_MCU/16000000 * 1000000UL/F_DEV);  // Td = 1/(4 * Fdev) phase-shift   https://tj-lab.org/2020/08/27/si5351%e5%8d%98%e4%bd%93%e3%81%a73mhz%e4%bb%a5%e4%b8%8b%e3%81%ae%e7%9b%b4%e4%ba%a4%e4%bf%a1%e5%8f%b7%e3%82%92%e5%87%ba%e5%8a%9b%e3%81%99%e3%82%8b/
      ms(MS1,  fvcoa, fout, PLLA, 0, 0, rdiv);
      oe(0b00000011);  // output enable CLK0, CLK1
#endif
      _fout = fout;  // cache
      _div = d;
      _msa128min512 = fvcoa / fxtal * 128 - 512;
      _msb128=((uint64_t)(fvcoa % fxtal)*_MSC*128) / fxtal;
      //_mod = fvcoa % fxtal;
  }

  void freqb(uint32_t fout){  // Set a CLK2 to fout Hz (on PLLB)
      uint16_t d = (16 * fxtal) / fout;
      if(d % 2) d++;  // even numbers preferred for divider (AN619 p.4 and p.6)
      uint32_t fvcoa = d * fout;  // Variable PLLA VCO frequency at integer multiple of fout at around 27MHz*16 = 432MHz

      ms(MSNB, fvcoa, fxtal);
      ms(MS2,  fvcoa, fout, PLLB, 0, 0, 0);
      //ms(MS1,  fvcoa, fout, PLLB, 0, 0, 0); // temp hack - put on CLK1 instead
      //Should also call oe() to enable CLK2 too. 
  }
  
  void powerDown(){
    SendRegister(3, 0b11111111); // Disable all CLK outputs
    SendRegister(24, 0b00010000); // Disable state: CLK2 HIGH state, CLK0 & CLK1 LOW state when disabled; CLK2 needs to be in HIGH state to make sure that cap to gate is already charged, preventing "exponential pulse is caused by CLK2, which had been at 0v whilst it was disabled, suddenly generating a 5vpp waveform, which is “added to” the 0v filtered PWM output and causing the output fets to be driven with the full 5v pp.", see: https://forum.dl2man.de/viewtopic.php?t=146&p=1307#p1307
    SendRegister(25, 0b00000000); // Disable state: LOW state when disabled
    for (int addr = 16; addr != 24; addr++) 
      SendRegister(addr, 0b10000000);  // Conserve power when output is disabled
    SendRegister(187, 0);        // Disable fanout (power-safe)
    // To initialise things as they should:
    SendRegister(149, 0);        // Disable spread spectrum enable
    SendRegister(183, 0b11010010);  // Internal CL = 10 pF (default)
  }

  uint16_t BERTest(int32_t f) {
    // Measure I2C Bit-Error Rate (BER); should be error free for a thousand random bulk PLLB writes
    freq(f, 0, 90);  // freq needs to be set in order to use freq_calc_fast()
    uint16_t i2c_error = 0;  // number of I2C byte transfer errors
    for(int i = 0; i != 1000; i++){
      freq_calc_fast(i);
      //for(int j = 0; j != 8; j++) si5351.pll_regs[j] = rand();
      SendPLLRegisterBulk();
      #define SI_SYNTH_PLL_A 26
      for(int j = 4; j != 8; j++)
        if(RecvRegister(SI_SYNTH_PLL_A + j) != pll_regs[j])
          i2c_error++;

    }
    return i2c_error;
  }

  void speedTest(int32_t f, int n) {  // measure time to update the delta f
    freq(f, 0, 90);  // freq needs to be set in order to use freq_calc_fast();
    for (int i=0; i<n; i++) {
      freq_calc_fast(i);
      SendPLLRegisterBulk();
    }
  }

  // shims to simplify integration with the T41 code
  //


  void reset(void) { Serial.println("si5351 reset"); Wire.begin(); powerDown(); }
  bool init(uint8_t xtal_load_c, uint32_t xo_freq, int32_t corr) { Serial.println("si5351 init");powerDown(); return false; }
  void set_ms_source(enum si5351_clock clk, enum si5351_pll pll) { }
  void drive_strength(enum si5351_clock clk, enum si5351_drive drive) { }
  uint8_t set_freq(uint64_t f, enum si5351_clock clk) { Serial.print("set_freq "); Serial.println(f); freq(f, 0, 90); return 0; }
  void pll_reset(enum si5351_pll target_pll) { }
  uint8_t set_freq_manual(uint64_t f, uint64_t pll_freq, enum si5351_clock clk) { 
    
    freq(f, 0, 90); 
    Serial.print("set_freq_man "); Serial.println(f); 
    return 0; 
  } // used by bode plot
  void set_phase(enum si5351_clock clk, uint8_t phase) { }
  void output_enable(enum si5351_clock clk, uint8_t enable) { }
  

};

//Si5351 si5351;
