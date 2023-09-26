#include <stdint.h>
#include <array>

namespace UNITREE_LEGGED_SDK{
    typedef struct{
        uint8_t R1          :1;
        uint8_t L1          :1;
        uint8_t start       :1;
        uint8_t select      :1;
        uint8_t R2          :1;
        uint8_t L2          :1;
        uint8_t F1          :1;
        uint8_t F2          :1;
        uint8_t A           :1;
        uint8_t B           :1;
        uint8_t X           :1;
        uint8_t Y           :1;
        uint8_t up          :1;
        uint8_t right       :1;
        uint8_t down        :1;
        uint8_t left        :1;


        uint8_t get_R1()     const { return R1;       }
        uint8_t get_L1()     const { return L1;       }
        uint8_t get_start()  const { return start;    }
        uint8_t get_select() const { return select;   }
        uint8_t get_R2()     const { return R2;       }
        uint8_t get_L2()     const { return L2;       }
        uint8_t get_F1()     const { return F1;       }
        uint8_t get_F2()     const { return F2;       }
        uint8_t get_A()      const { return A;        }
        uint8_t get_B()      const { return B;        }
        uint8_t get_X()      const { return X;        }
        uint8_t get_Y()      const { return Y;        }
        uint8_t get_up()     const { return up;       }
        uint8_t get_right()  const { return right;    }
        uint8_t get_down()   const { return down;     }
        uint8_t get_left()   const { return left;     }

        void set_R1(uint8_t value) { R1 = value; }
        void set_L1(uint8_t value) { L1 = value; }
        void set_start(uint8_t value) { start = value; }
        void set_select(uint8_t value) { select = value; }
        void set_R2(uint8_t value) { R2 = value; }
        void set_L2(uint8_t value) { L2 = value; }
        void set_F1(uint8_t value) { F1 = value; }
        void set_F2(uint8_t value) { F2 = value; }
        void set_A(uint8_t value) { A = value; }
        void set_B(uint8_t value) { B = value; }
        void set_X(uint8_t value) { X = value; }
        void set_Y(uint8_t value) { Y = value; }
        void set_up(uint8_t value) { up = value; }
        void set_right(uint8_t value) { right = value; }
        void set_down(uint8_t value) { down = value; }
        void set_left(uint8_t value) { left = value; }
        
    } Components;

    typedef union {
        Components components;
        uint16_t value;
    } xKeySwitchUnion;

    typedef struct {
        std::array<uint8_t, 2> head;
        xKeySwitchUnion btn;
        float lx;
        float rx;
        float ry;
        float reserve;
        float ly;
        std::array<uint8_t, 16> idle;
    } xRockerBtnDataStruct;

    xRockerBtnDataStruct parse_joystick_data(std::array<uint8_t, 40> data){
        xRockerBtnDataStruct joystick_data;
        memcpy(&joystick_data, &data, 40);
        return joystick_data;
    }

    std::array<uint8_t, 40> compress_joystick_data(xRockerBtnDataStruct joystick_data){
        std::array<uint8_t, 40> data;
        memcpy(&data, &joystick_data, 40);
        return data;
    }
    
}

