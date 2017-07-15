Component version 1.0 developed by e2forlife.

# W5500:

Ethernet controller from WizNet, controlled via SPI in mode 0 and 3 (this component works on SPI mode 0).

# TODO for version 1.1:

- [ ] Support for VLD and FLD transactions, choosing one of them on the component customizer.
- [ ] Add support for Multi-cast, as pointed out in the udp.c file.
- [ ] Update to C99 where possible.
- [x] Split x_Send function into x_Read and x_Write functions.
- [ ] Enums instead of defines where possible.
- [x] Header file for each source file.
- [ ] Update Doxygen documentation.
- [x] Start the SPI component and set /CSN pin high inside the x_Start function.
