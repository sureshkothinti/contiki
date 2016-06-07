#undef UIP_CONF_BUFFER_SIZE
#define UIP_CONF_BUFFER_SIZE           1280

#undef REST_MAX_CHUNK_SIZE
#define REST_MAX_CHUNK_SIZE            256

//  srf06 spi pins
#define BOARD_IOID_FLASH_CS       IOID_14
#define BOARD_FLASH_CS            (1 << BOARD_IOID_FLASH_CS)
#define BOARD_IOID_SPI_CLK_FLASH  IOID_10
#define BOARD_IOID_SPI_MOSI       IOID_9
#define BOARD_IOID_SPI_MISO       IOID_8
