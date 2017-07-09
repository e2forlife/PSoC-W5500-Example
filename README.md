Component version 1.0 developed by e2forlife.

# W5500:

Ethernet controller from WizNet, controlled via SPI in mode 0 and 3 (this component works on SPI mode 0).

# TODO for version 1.1:

- [ ] Version 1.0 of the component only supported VLD SPI transactions, add suport for FLD transactions and add an option on the customizer to choose one.
- [ ] Multi-cast, as pointed out in the udp.c file.
- [ ] Update to C99.
- [x] Split x_Send function in x_Read and x_Write functions.
- [ ] Enums instead of defines.
- [x] Header file for each source file.
- [ ] Update Doxygen documentation.
- [x] Start the SPI component and set /CSN pin high in x_Start function.
