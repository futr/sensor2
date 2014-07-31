#include "fifo.h"

void fifo_init( FIFO *fifo, void *data, size_t data_size, size_t pad, size_t length )
{
    /* 初期化とフラッシュ */
    fifo->data      = data;
    fifo->data_size = data_size;
    fifo->length    = length;
    fifo->pad       = pad;
    fifo->end       = (char *)data + ( length - 1 ) * ( pad + data_size );
    fifo->level     = 0;

    /* ポインター初期化 */
    fifo->w = data;
    fifo->r = data;
}

char fifo_read( FIFO *fifo, void *dest )
{
    /* 読み込み */

    /* 書きに追いついていないか */
    if ( fifo->w == fifo->r ) {
        return 0;
    }

    /* 読み込み */
    memcpy( dest, fifo->r, fifo->data_size );

    /* ポインターをすすめる */
    if ( fifo->r < fifo->end ) {
        /* すすめる */
        fifo->r += fifo->pad + fifo->data_size;
    } else {
        /* 先頭に戻る */
        fifo->r = fifo->data;
    }

    fifo->level--;

    return 1;
}

char fifo_write( FIFO *fifo, void *source )
{
    /* 書き込み */
    unsigned char *next_w;

    /* 次の座標を計算しておく */
    if ( fifo->w < fifo->end ) {
        next_w = fifo->w + fifo->pad + fifo->data_size;
    } else {
        next_w = fifo->data;
    }

    /* 書き込み ( 仮にオーバーフローしていても書き込む ) */
    memcpy( fifo->w, source, fifo->data_size );

    /* 読み込みに追いついていたら進めない */
    if ( next_w != fifo->r ) {
        /* ポインターをすすめる */
        fifo->w = next_w;
    }

    /* レベル制御 */
    if ( fifo->level < fifo->length ) {
        fifo->level++;
    }

    return 1;
}


char fifo_can_read( FIFO *fifo )
{
    /* 読み込めるか */
    if ( fifo->w == fifo->r ) {
        return 0;
    } else {
        return 1;
    }
}

char fifo_can_write( FIFO *fifo )
{
    /* 書き込めるか */
    if ( fifo_level( fifo ) < fifo->length ) {
        return 1;
    } else {
        return 0;
    }
}

size_t fifo_level( FIFO *fifo )
{
    /* 現在のデーター数を返す */
    return fifo->level;
}
