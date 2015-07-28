#pragma once

#include "robot.hpp"

class System
{
  public:
    System(void);
    ~System(void);


    /**
     * [System::Init Setup the FPGA interface]
     * @return  [The initialization error code.]
     */
    ERR_t Init(void);

    void Start(void);
    bool SelfTest(void);
    long GetTime(void);
    bool CanDrive(void);
    bool CanDribble(void);

  protected:

  private:
    bool isInit;
};
