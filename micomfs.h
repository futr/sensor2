/*
 * マイコン用セクタ単位ファイルシステム
 *
 * FATなし、削除・拡張不可能
 * 読み込み・書きこみ単位はセクターのみ
 *
 * --- 制約 ---
 *
 * fcreateでファイルを作成した場合、そのファイルをfcloseするまで
 * 新たにfcreateしてはいけません。
 * もしした場合、かつ最初のファイルで予約したセクター数以上の書き込みが
 * 最初のファイルで発生すると、次のファイルが壊れます。
 * 逆に予約セクター数以内であれば問題ありません。
 *
 */

#ifndef MICOMFS_H_INCLUDED
#define MICOMFS_H_INCLUDED

/*
 * パソコンなどのスペックに余裕があるデバイスで，
 * MICOMFS_ENABLE_EXFUNCTIONS
 * を有効にすると
 * char micomfs_fdelete( MicomFS *fs, const char *name );
 * char micomfs_clean_fs( MicomFS *fs );
 * char micomfs_get_file_list( MicomFS *fs, MicomFSFile **list, uint16_t *count );
 * が利用可能になります
 * ( clean_fsとfdeleteはまだ実装していません )
 *
 */

// #define MICOMFS_ENABLE_EXFUNCTIONS

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#define MICOMFS_API_VERSION_CODE "0.1"
#define MICOMFS_SIGNATURE 0x5E
#define MICOMFS_MAX_FILE_SECOTR_COUNT 0xFFFFFFFF
#define MICOMFS_MAX_FILE_NAME_LENGTH  128

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MicomFSDeviceAuto,
    MicomFSDeviceSD,
    MicomFSDeviceFile,
    MicomFSDeviceWinDrive,
} MicomFSDeviceType;

typedef enum {
    MicomFSDeviceModeRead,
    MicomFSDeviceModeWrite,
    MicomFSDeviceModeReadWrite,
} MicomFSDeviceMode;

typedef enum {
    MicomFSReturnFalse    = 0,
    MicomFSReturnTrue     = 1,
    MicomFSReturnSameName = 2,
} MicomFSReturn;

typedef enum {
    MicomFSFileModeRead,
    MicomFSFileModeWrite,
    MicomFSFileModeReadWrite,
} MicomFSFileMode;

typedef enum {
    MicomFSFileStatusStop,
    MicomFSFileStatusRead,
    MicomFSFileStatusWrite,
    MicomFSFileStatusError,
} MicomFSFileStatus;

typedef enum {
    MicomFSFileFlagUnknown = 0x00,
    MicomFSFileFlagNormal  = 0xAB,
    MicomFSFileFlagDeleted = 0xF3,
} MicomFSFileFlag;

typedef struct {
    uint16_t dev_sector_size;       /* デバイス上のセクターサイズ */
    uint32_t dev_sector_count;      /* デバイス上のセクター数 */

    uint16_t sector_size;           /* セクターの大きさ 512 */
    uint32_t sector_count;          /* セクター数 ( 先頭セクター含む ) */
    uint16_t entry_count;           /* 全エントリー数 */
    uint16_t used_entry_count;      /* 使用済みエントリー数 */

    /* PCなど用拡張情報 */
    void *device;
    char *dev_name;
    uint32_t dev_current_sector;
    uint16_t dev_current_spos;
    MicomFSDeviceType dev_type;
} MicomFS;

typedef struct {
    MicomFS *fs;
    uint32_t current_sector;    /* アクセス中セクタ番号 */
    uint16_t spos;              /* セクター内位置 */
    MicomFSFileStatus status;   /* アクセス状態 */
    MicomFSFileMode mode; /* Access mode */

    uint16_t entry_id;          /* このエントリのセクタ番号 */
    uint8_t  flag;              /* フラグ */
    uint32_t start_sector;      /* ファイルの開始セクタ */
    uint32_t sector_count;      /* ファイルのセクター数 */
    uint32_t max_sector_count;  /* ファイルの最大セクター数 */

    char *name;                 /* ファイル名 */
} MicomFSFile;

char micomfs_open_device( MicomFS *fs, const char *dev_name, MicomFSDeviceType dev_type, MicomFSDeviceMode mode );
char micomfs_close_device( MicomFS *fs );

char micomfs_init_fs( MicomFS *fs );
char micomfs_format( MicomFS *fs, uint16_t sector_size, uint32_t sector_count, uint16_t entry_count, uint16_t used_entry_count );

char micomfs_fcreate( MicomFS *fs, MicomFSFile *fp, char *name, uint32_t reserved_sector_count );
char micomfs_fopen(MicomFS *fs, MicomFSFile *fp, MicomFSFileMode mode, char *name );
char micomfs_fclose( MicomFSFile *fp );

char micomfs_start_fwrite( MicomFSFile *fp, uint32_t sector );
char micomfs_start_fread( MicomFSFile *fp, uint32_t sector );
char micomfs_fwrite( MicomFSFile *fp, const void *src, uint16_t count );
char micomfs_fread( MicomFSFile *fp, void *dest, uint16_t count );
char micomfs_stop_fwrite( MicomFSFile *fp, uint8_t fill );
char micomfs_stop_fread( MicomFSFile *fp );

char micomfs_seq_fwrite( MicomFSFile *fp, const void *src, uint16_t count );
char micomfs_seq_fread( MicomFSFile *fp, void *dest, uint16_t count );

uint16_t micomfs_get_file_spos( MicomFSFile *fp );
uint32_t micomfs_get_file_current_sector( MicomFSFile *fp );

char micomfs_read_entry( MicomFS *fs, MicomFSFile *fp, uint16_t entry_id, const char *name );
char micomfs_write_entry( MicomFSFile *fp );

/* 拡張機能 */
#ifdef MICOMFS_ENABLE_EXFUNCTIONS
char micomfs_fdelete( MicomFS *fs, const char *name );
char micomfs_clean_fs( MicomFS *fs );
char micomfs_get_file_list( MicomFS *fs, MicomFSFile **list, uint16_t *count );
#endif

#ifdef __cplusplus
}
#endif

#endif
