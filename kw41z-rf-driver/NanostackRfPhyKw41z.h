/*
 * 
 */

#ifndef NANOSTACK_PHY_KW41Z_H_
#define NANOSTACK_PHY_KW41Z_H_

#include "mbed.h"
#include "NanostackRfPhy.h"

class NanostackRfPhyKw41z : public NanostackRfPhy {
public:
    NanostackRfPhyKw41z();
    ~NanostackRfPhyKw41z();
    int8_t rf_register();
    void rf_unregister();
    void get_mac_address(uint8_t *mac);
    void set_mac_address(uint8_t *mac);
};

#endif /* NANOSTACK_PHY_NCS36510_H_ */
