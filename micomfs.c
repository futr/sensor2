#include "micomfs.h"
#include "micomfs_dev.h"

char micomfs_init_fs( MicomFS *fs )
{
    /* デバイスにアクセスしてFS初期化 */
    uint8_t signature;
    uint8_t data;
    uint32_t i;

    /* デバイス上のセクター数とセクターサイズを取得 */
    if ( !micomfs_dev_get_info( &fs->dev_sector_size, &fs->dev_sector_count ) ) {
        return 0;
    }

    /* デバイスの先頭セクターにアクセスしてファイルシステムの情報取得 */
    micomfs_dev_start_read( fs, 0 );

    micomfs_dev_read( fs, &signature, 1 );
    micomfs_dev_read( fs, &fs->sector_size, 2 );
    micomfs_dev_read( fs, &fs->sector_count, 4 );
    micomfs_dev_read( fs, &fs->entry_count, 2 );
    micomfs_dev_read( fs, &fs->used_entry_count, 2 );

    for ( i = 0; i < fs->dev_sector_size - 11; i++ ) {
        micomfs_dev_read( fs, &data, 1 );
    }

    micomfs_dev_stop_read( fs );

    /* シグネチャ・セクタサイズが対応不可能だったら失敗 */
    if ( signature != MICOMFS_SIGNATURE || fs->dev_sector_size != fs->sector_size ) {
        return 0;
    }

    return 1;
}

char micomfs_format( MicomFS *fs, uint16_t sector_size, uint32_t sector_count, uint16_t entry_count, uint16_t used_entry_count )
{
    /* デバイスをフォーマットする */
    uint8_t signature = MICOMFS_SIGNATURE;
    uint8_t data;
    uint32_t i;

    /* デバイス上のセクター数とセクターサイズを取得 */
    if ( !micomfs_dev_get_info( &fs->dev_sector_size, &fs->dev_sector_count ) ) {
        return 0;
    }

    /* 構造体初期化 */
    fs->sector_size  = sector_size;
    fs->sector_count = sector_count;
    fs->entry_count  = entry_count;
    fs->used_entry_count = used_entry_count;

    /* 指定されたセクターサイズとデバイスセクターサイズが異なれば失敗 */
    if ( fs->dev_sector_size != fs->sector_size ) {
        return 0;
    }

    /* デバイスの先頭セクターにファイルシステム書き込み */
    micomfs_dev_start_write( fs, 0 );

    micomfs_dev_write( fs, &signature, 1 );
    micomfs_dev_write( fs, &fs->sector_size, 2 );
    micomfs_dev_write( fs, &fs->sector_count, 4 );
    micomfs_dev_write( fs, &fs->entry_count, 2 );
    micomfs_dev_write( fs, &fs->used_entry_count, 2 );

    data = 0;

    for ( i = 0; i < fs->dev_sector_size - 11; i++ ) {
        micomfs_dev_write( fs, &data, 1 );
    }

    micomfs_dev_stop_write( fs );

    return 1;
}

char micomfs_fcreate( MicomFS *fs, MicomFSFile *fp, const char *name )
{
    /* ファイル新規作成 ファイルは最小１セクタ消費する */
    MicomFSFile last;

    /* エントリーが追加できなければ失敗 */
    if ( fs->entry_count <= fs->used_entry_count ) {
        return 0;
    }

    /* 存在すれば現在の最終ファイル取得 */
    if ( fs->used_entry_count ) {
        if ( !micomfs_read_entry( fs, &last, fs->used_entry_count - 1, NULL ) ) {
            return 0;
        }

        /* セクターが余ってなければ失敗 */
        if ( fs->sector_count - ( last.start_sector + last.sector_count ) < 1 ) {
            return 0;
        }

        /* 情報作成 */
        fp->max_sector_count = fs->sector_count - ( last.start_sector + last.sector_count );
        fp->entry_id = last.entry_id + 1;
        fp->start_sector = last.start_sector + last.sector_count;
    } else {
        /* 情報作成 */
        fp->max_sector_count = fs->sector_count - ( 1 + fs->entry_count );
        fp->entry_id = 0;
        fp->start_sector = 1 + fs->entry_count;
    }

    /* ファイル情報作成 */
    fp->current_sector = 0;
    fp->flag = MicomFSFileFlagNormal;
    fp->fs = fs;
    fp->sector_count = 1;
    fp->current_sector = 0;
    fp->spos = 0;
    fp->status = MicomFSFileStatusStop;
    fp->name = name;

    /* エントリー書き出し */
    micomfs_write_entry( fp );

    /* エントリー追加 */
    fs->used_entry_count++;

    /* 先頭をフォーマット ( 更新を書き込み ) */
    micomfs_format( fs, fs->sector_size, fs->sector_count, fs->entry_count, fs->used_entry_count );

    return 1;
}

char micomfs_read_entry( MicomFS *fs, MicomFSFile *fp, uint16_t entry_id, const char *name )
{
    /* 指定IDのエントリー情報を取得 nameがNULLでなければ一致すればMicomFSReturnSameNameが帰る */
    uint32_t i;
    uint8_t data;
    char same = 0;

    /* 指定エントリーにアクセス開始 */
    if ( !micomfs_dev_start_read( fs, entry_id + 1 ) ) {
        return 0;
    }

    micomfs_dev_read( fs, &fp->flag, 1 );
    micomfs_dev_read( fs, &fp->start_sector, 4 );
    micomfs_dev_read( fs, &fp->sector_count, 4 );

    /* 名前があれば一致確認 */
    if ( name != NULL ) {
        /* 名前一致フラグON */
        same = 1;

        for ( i = 0; i < strlen( name ) + 1; i++ ) {
            micomfs_dev_read( fs, &data, 1 );

            /* 一致しなければ一致フラグOFF */
            if ( data != name[i] ) {
                same = 0;
            }
        }
    }

    /* 残り */
    for ( i = 0; i < fs->sector_size - ( 9 + strlen( name ) + 1 ); i++ ) {
        micomfs_dev_read( fs, &data, 1 );
    }

    micomfs_dev_stop_read( fs );

    /* エントリー情報補正 */
    fp->fs = fs;
    fp->max_sector_count = fp->sector_count;        /* 最大セクター数は最終ファイル以外現在のセクター数 */
    fp->spos = 0;
    fp->current_sector = 0;
    fp->entry_id = entry_id;
    fp->status = MicomFSFileStatusStop;

    if ( same ) {
        return MicomFSReturnSameName;
    } else {
        return MicomFSReturnTrue;
    }
}

char micomfs_write_entry( MicomFSFile *fp )
{
    /* エントリー書き出し */
    uint32_t i;
    uint8_t data;

    /* 指定エントリーにアクセス開始 */
    if ( !micomfs_dev_start_write( fp->fs, fp->entry_id + 1 ) ) {
        return 0;
    }

    micomfs_dev_write( fp->fs, &fp->flag, 1 );
    micomfs_dev_write( fp->fs, &fp->start_sector, 4 );
    micomfs_dev_write( fp->fs, &fp->sector_count, 4 );
    micomfs_dev_write( fp->fs, fp->name, strlen( fp->name ) + 1 );

    for ( i = 0; i < fp->fs->sector_size - ( 9 + strlen( fp->name ) + 1 ); i++ ) {
        micomfs_dev_write( fp->fs, &data, 1 );
    }

    micomfs_dev_stop_write( fp->fs );

    return 1;
}


char micomfs_fopen( MicomFS *fs, MicomFSFile *fp, const char *name )
{
    /* ファイルを開く */
    uint32_t i;

    /* 指定ファイル名と一致するエントリを探る */
    for ( i = 0; i < fs->used_entry_count; i++ ) {
        /* iエントリー読み込み */
        if ( micomfs_read_entry( fs, fp, i, name ) == MicomFSReturnSameName ) {
            break;
        }
    }

    /* 最後まで到達してれば失敗 */
    if ( i >= fs->used_entry_count ) {
        return 0;
    }

    return 1;
}

char micomfs_fclose( MicomFSFile *fp )
{
    /* ファイルアクセス終了 */

    /* もしアクセス中なら停止させる */
    if ( fp->status == MicomFSFileStatusRead ) {
        micomfs_stop_fread( fp );
    } else if ( fp->status == MicomFSFileStatusRead ) {
        micomfs_stop_fwrite( fp, 0 );
    }

    /* エントリーを書き込む */
    micomfs_write_entry( fp );

    /* ファイル情報操作 */
    fp->status = MicomFSFileStatusStop;

    return 1;
}

char micomfs_start_fwrite( MicomFSFile *fp, uint32_t sector )
{
    /* 書きこみ開始 */

    /* 最大セクター数を超えてたら失敗 */
    if ( fp->max_sector_count >= sector ) {
        return 0;
    }

    /* 指定セクターが現在のセクター数を超えていれば拡張 */
    if ( fp->sector_count >= sector ) {
        fp->sector_count = sector + 1;
    }

    /* カーソル設定 */
    fp->current_sector = sector;
    fp->spos = 0;

    /* 開始 */
    micomfs_dev_start_write( fp->fs, sector + fp->start_sector );

    return 1;
}

char micomfs_start_fread( MicomFSFile *fp, uint32_t sector )
{
    /* 読み込み開始 */

    /* セクター数を超えてたら失敗 */
    if ( fp->sector_count >= sector ) {
        return 0;
    }

    /* カーソル設定 */
    fp->current_sector = sector;
    fp->spos = 0;

    /* 開始 */
    micomfs_dev_start_read( fp->fs, sector + fp->start_sector );

    return 1;
}

char micomfs_fwrite( MicomFSFile *fp, const void *src, uint16_t count )
{
    /* 書き */
    micomfs_dev_write( fp->fs, src, count );

    /* カーソル進む */
    fp->spos += count;

    return 1;
}

char micomfs_fread( MicomFSFile *fp, void *dest, uint16_t count )
{
    /* 読む */
    micomfs_dev_read( fp->fs, dest, count );

    /* カーソル進む */
    fp->spos += count;

    return 1;
}

char micomfs_stop_fwrite( MicomFSFile *fp, uint8_t fill )
{
    /* 下記を終了 */
    uint32_t i;
    uint8_t data;

    /* まだ書き残しがあればfillで埋める */
    for ( i = fp->spos; i < fp->fs->sector_size; i++ ) {
        micomfs_dev_write( fp->fs, &data, 1 );
    }

    /* 書き終了 */
    micomfs_dev_stop_write( fp->fs );

    return 1;
}

char micomfs_stop_fread( MicomFSFile *fp )
{
    /* 読みを終了 */
    uint32_t i;
    uint8_t data;

    /* まだ読み残しがあれば捨てる */
    for ( i = fp->spos; i < fp->fs->sector_size; i++ ) {
        micomfs_dev_read( fp->fs, &data, 1 );
    }

    /* 読み終了 */
    micomfs_dev_stop_read( fp->fs );

    return 1;
}

uint16_t micomfs_get_file_spos( MicomFSFile *fp )
{
    /* 現在のセクタ内位置取得 */
    return fp->spos;
}

uint32_t micomfs_get_file_current_sector( MicomFSFile *fp )
{
    /* 現在のアクセス中セクタ取得 */
    return fp->current_sector;
}
