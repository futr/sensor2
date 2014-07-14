/*
 * 簡易汎用FIFO
 *
 */

#ifndef FIFO_H_INCLUDED
#define FIFO_H_INCLUDED

#include "ide.h"
#include <string.h>

typedef struct FIFO_tag {
    volatile unsigned char *data;       /* データー領域へのポインター */
    volatile unsigned char *r;          /* 次に読むアドレス */
    volatile unsigned char *w;          /* 次に書くアドレス */
    volatile unsigned char *end;        /* FIFOの終了地点 */

    size_t length;      /* データ個数 */
    size_t data_size;   /* データ一個のバイト数 */
    size_t pad;         /* パディングバイト数 */

    size_t level;       /* 満たされてる数 */
} FIFO;

#ifdef __cplusplus
extern "C" {
#endif

/* 初期化・フラッシュ */
void fifo_init( FIFO *fifo, void *data, size_t data_size, size_t pad, size_t length );

/* 読み書き */
char fifo_read( FIFO *fifo, void *dest );
char fifo_write( FIFO *fifo, void *source );

/* 情報 */
char fifo_can_read( FIFO *fifo );
char fifo_can_write( FIFO *fifo );
size_t fifo_level( FIFO *fifo );

#ifdef __cplusplus
}
#endif

#endif
