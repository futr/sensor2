/*
 * マイコン用セクタ単位ファイルシステム
 *
 * FATなし、削除・拡張不可能
 * 読み込み・書きこみ単位はセクターのみ
 *
 * セクターアドレスはFS上の「仮想」セクターです。
 * つまり、デバイス上のセクターサイズとFS上のセクターサイズは独立していてもよい
 * のですが、このプログラムではサポートしてません。
 *
 * ファイルシステムと言いつつ、セクターアクセスがもろにユーザーに漏洩しています。
 * ファイル操作関数のセクターとはファイル内セクターのことです。
 * ファイルの先頭をオフセットとしたセクターアドレスと考えてください。
 *
 * ファイル操作関数は、セクターサイズ以上のアクセスを行うと単純に失敗します。
 * アクセス終了作業は行わないので、確実にアクセス終了関数を呼んでください。
 * アクセス途中に終了関数を呼ぶと、自動的に残りのデーターを読み捨て・書き込みします。
 *
 * 普通にPetit FatFsを利用したほうがよいと思います。
 *
 * ファイル名操作が雑なので、おかしなポインターを渡すとクラッシュするかもしれないです。
 *
 * ファイル情報破壊を阻止するため、定期的に
 * micomfs_write_entry( MicomFSFile *fp );
 * を実行してください
 *
 * --- 制約 ---
 *
 * fcreateであるファイルを作成した場合、そのファイルをfcloseするまで
 * 新たにfcreateしてはいけません。
 * もしした場合、かつ最初のファイルで予約したセクター数以上の書き込みが
 * 最初のファイルで発生すると、次のファイルが壊れます。
 * 逆に予約セクター数以内であれば問題ありません。
 *
 *
 */


#ifndef MICOMFS_H_INCLUDED
#define MICOMFS_H_INCLUDED

#include <stdio.h>
#include <string.h>

#define MICOMFS_SIGNATURE 0x5E
#define MICOMFS_MAX_FILE_SECOTR_COUNT 0xFFFFFFFF

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MicomFSReturnFalse    = 0,
    MicomFSReturnTrue     = 1,
    MicomFSReturnSameName = 2,
} MicomFSReturn;

/*
typedef enum {
    MicomFSFileModeRead,
    MicomFSFileModeWrite,
    MicomFSFileModeReadWrite,
} MicomFSFileMode;
*/

typedef enum {
    MicomFSFileAccessModeNormal       = 1 << 0,
    MicomFSFileAccessModeAutoStop     = 1 << 1,
    MicomFSFileAccessModeAutoContinue = 1 << 2,
} MicomFSFileAccessMode;

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
} MicomFS;

typedef struct {
    MicomFS *fs;
    uint32_t current_sector;    /* アクセス中セクタ番号 */
    uint16_t spos;              /* セクター内位置 */
    MicomFSFileStatus status;   /* アクセス状態 */

    uint16_t entry_id;          /* このエントリのセクタ番号 */
    uint8_t  flag;              /* フラグ */
    uint32_t start_sector;      /* ファイルの開始セクタ */
    uint32_t sector_count;      /* ファイルのセクター数 */
    uint32_t max_sector_count;  /* ファイルの最大セクター数 */

    const char *name;           /* ファイル名 : ポインタしか保持しない！ */
} MicomFSFile;

char micomfs_init_fs( MicomFS *fs );
char micomfs_format( MicomFS *fs, uint16_t sector_size, uint32_t sector_count, uint16_t entry_count, uint16_t used_entry_count );

char micomfs_fcreate( MicomFS *fs, MicomFSFile *fp, const char *name, uint32_t reserved_sector_count );
char micomfs_fopen( MicomFS *fs, MicomFSFile *fp, const char *name );
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

/* 以下PCとか大富豪用 削除済みファイルシフトも必要？ */
/*
char micomfs_fdelete( MicomFS *fs, const char *name );
char micomfs_clean( MicomFS *fs );
char micomfs_get_file_list( MicomFS *fs, MicomFSFile **list, uint16_t count );
*/

#ifdef __cplusplus
}
#endif

#endif
