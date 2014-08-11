#ifndef MICOMFS_DEV_H_INCLUDED
#define MICOMFS_DEV_H_INCLUDED

#include <stdio.h>
#include "micomfs.h"

#ifdef __cplusplus
extern "C" {
#endif

char micomfs_dev_get_info( MicomFS *fs, uint16_t *sector_size, uint32_t *sector_count );
char micomfs_dev_open( MicomFS *fs, const char *dev_name, MicomFSDeviceType dev_type );
char micomfs_dev_close( MicomFS *fs );

char micomfs_dev_start_write( MicomFS *fs, uint32_t sector );
char micomfs_dev_write( MicomFS *fs, const void *src, uint16_t count );
char micomfs_dev_stop_write( MicomFS *fs );
char micomfs_dev_start_read( MicomFS *fs, uint32_t sector );
char micomfs_dev_read( MicomFS *fs, void *dest, uint16_t count );
char micomfs_dev_stop_read( MicomFS *fs );

#ifdef __cplusplus
}
#endif

#endif
