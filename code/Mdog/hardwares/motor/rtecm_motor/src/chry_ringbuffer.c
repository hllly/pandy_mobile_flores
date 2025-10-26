/*
 * Copyright (c) 2022, Egahp
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "chry_ringbuffer.h"

/*****************************************************************************
* @brief        初始化环形缓冲区
* 
* @param[in]    rb          环形缓冲区实例
* @param[in]    pool        内存池地址
* @param[in]    size        内存大小（字节），
*                           必须是 2 的幂
* 
* @retval int               0：成功 -1：错误
*****************************************************************************/
int chry_ringbuffer_init(chry_ringbuffer_t *rb, void *pool, uint32_t size)
{
    if (NULL == rb) {
        return -1;
    }

    if (NULL == pool) {
        return -1;
    }

    if ((size < 2) || (size & (size - 1))) {
        return -1;
    }

    rb->in = 0;
    rb->out = 0;
    rb->mask = size - 1;
    rb->pool = pool;

    return 0;
}

/*****************************************************************************
* @brief        重置环形缓冲区并清空所有数据，
*               多线程环境下应加锁
* 
* @param[in]    rb          环形缓冲区实例
* 
*****************************************************************************/
void chry_ringbuffer_reset(chry_ringbuffer_t *rb)
{
    rb->in = 0;
    rb->out = 0;
}

/*****************************************************************************
* @brief        重置环形缓冲区的读指针并清空数据，
*               多线程环境下应加锁，
*               单读线程可无需锁
* 
* @param[in]    rb          环形缓冲区实例
* 
*****************************************************************************/
void chry_ringbuffer_reset_read(chry_ringbuffer_t *rb)
{
    rb->out = rb->in;
}

/*****************************************************************************
* @brief        获取环形缓冲区的总容量（字节）
* 
* @param[in]    rb          环形缓冲区实例
* 
* @retval uint32_t          总容量（字节）
*****************************************************************************/
uint32_t chry_ringbuffer_get_size(chry_ringbuffer_t *rb)
{
    return rb->mask + 1;
}

/*****************************************************************************
* @brief        获取环形缓冲区已使用空间（字节）
* 
* @param[in]    rb          环形缓冲区实例
* 
* @retval uint32_t          已使用空间（字节）
*****************************************************************************/
uint32_t chry_ringbuffer_get_used(chry_ringbuffer_t *rb)
{
    return rb->in - rb->out;
}

/*****************************************************************************
* @brief        获取环形缓冲区剩余空间（字节）
* 
* @param[in]    rb          环形缓冲区实例
* 
* @retval uint32_t          剩余空间（字节）
*****************************************************************************/
uint32_t chry_ringbuffer_get_free(chry_ringbuffer_t *rb)
{
    return (rb->mask + 1) - (rb->in - rb->out);
}

/*****************************************************************************
* @brief        检查环形缓冲区是否已满
* 
* @param[in]    rb          环形缓冲区实例
* 
* @retval true              已满
* @retval false             未满
*****************************************************************************/
bool chry_ringbuffer_check_full(chry_ringbuffer_t *rb)
{
    return chry_ringbuffer_get_used(rb) > rb->mask;
}

/*****************************************************************************
* @brief        检查环形缓冲区是否为空
* 
* @param[in]    rb          环形缓冲区实例
* 
* @retval true              为空
* @retval false             非空
*****************************************************************************/
bool chry_ringbuffer_check_empty(chry_ringbuffer_t *rb)
{
    return rb->in == rb->out;
}

/*****************************************************************************
* @brief        写入一个字节，
*               多线程写入需加锁，
*               单写线程可无需锁
* 
* @param[in]    rb          环形缓冲区实例
* @param[in]    byte        待写入数据
* 
* @retval true              写入成功
* @retval false             环形缓冲区已满
*****************************************************************************/
bool chry_ringbuffer_write_byte(chry_ringbuffer_t *rb, uint8_t byte)
{
    if (chry_ringbuffer_check_full(rb)) {
        return false;
    }

    ((uint8_t *)(rb->pool))[rb->in & rb->mask] = byte;
    rb->in++;
    return true;
}

/*****************************************************************************
* @brief        覆写一个字节，并丢弃最旧的数据，
*               始终需要加锁
*
* @param[in]    rb          环形缓冲区实例
* @param[in]    byte        待写入数据
* 
* @retval true              写入成功
* @retval false             始终返回 true
*****************************************************************************/
bool chry_ringbuffer_overwrite_byte(chry_ringbuffer_t *rb, uint8_t byte)
{
    if (chry_ringbuffer_check_full(rb)) {
        rb->out++;
    }

    ((uint8_t *)(rb->pool))[rb->in & rb->mask] = byte;
    rb->in++;
    return true;
}

/*****************************************************************************
* @brief        预读一个字节，
*               多线程读取需加锁，
*               单读线程可无需锁
* 
* @param[in]    rb          环形缓冲区实例
* @param[in]    byte        保存数据的指针
* 
* @retval true              读取成功
* @retval false             环形缓冲区为空
*****************************************************************************/
bool chry_ringbuffer_peek_byte(chry_ringbuffer_t *rb, uint8_t *byte)
{
    if (chry_ringbuffer_check_empty(rb)) {
        return false;
    }

    *byte = ((uint8_t *)(rb->pool))[rb->out & rb->mask];
    return true;
}

/*****************************************************************************
* @brief        读取一个字节，
*               多线程读取需加锁，
*               单读线程可无需锁
* 
* @param[in]    rb          环形缓冲区实例
* @param[in]    byte        保存数据的指针
* 
* @retval true              读取成功
* @retval false             环形缓冲区为空
*****************************************************************************/
bool chry_ringbuffer_read_byte(chry_ringbuffer_t *rb, uint8_t *byte)
{
    bool ret;
    ret = chry_ringbuffer_peek_byte(rb, byte);
    rb->out += ret;
    return ret;
}

/*****************************************************************************
* @brief        丢弃环形缓冲区中的一个字节，
*               多线程读取需加锁，
*               单读线程可无需锁
* 
* @param[in]    rb          环形缓冲区实例
* 
* @retval true              操作成功
* @retval false             环形缓冲区为空
*****************************************************************************/
bool chry_ringbuffer_drop_byte(chry_ringbuffer_t *rb)
{
    if (chry_ringbuffer_check_empty(rb)) {
        return false;
    }

    rb->out += 1;
    return true;
}

/*****************************************************************************
* @brief        向环形缓冲区写入数据，
*               多线程写入需加锁，
*               单写线程可无需锁
* 
* @param[in]    rb          环形缓冲区实例
* @param[in]    data        数据指针
* @param[in]    size        数据长度（字节）
* 
* @retval uint32_t          实际写入的字节数
*****************************************************************************/
uint32_t chry_ringbuffer_write(chry_ringbuffer_t *rb, void *data, uint32_t size)
{
    uint32_t unused;
    uint32_t offset;
    uint32_t remain;

    unused = (rb->mask + 1) - (rb->in - rb->out);

    if (size > unused) {
        size = unused;
    }

    offset = rb->in & rb->mask;

    remain = rb->mask + 1 - offset;
    remain = remain > size ? size : remain;

    memcpy(((uint8_t *)(rb->pool)) + offset, data, remain);
    memcpy(rb->pool, (uint8_t *)data + remain, size - remain);

    rb->in += size;

    return size;
}

/*****************************************************************************
* @brief        向环形缓冲区写入数据（会覆写最旧数据），
*               始终需要加锁
* 
* @param[in]    rb          环形缓冲区实例
* @param[in]    data        数据指针
* @param[in]    size        数据长度（字节）
* 
* @retval uint32_t          实际写入的字节数
*****************************************************************************/
uint32_t chry_ringbuffer_overwrite(chry_ringbuffer_t *rb, void *data, uint32_t size)
{
    uint32_t unused;
    uint32_t offset;
    uint32_t remain;

    unused = (rb->mask + 1) - (rb->in - rb->out);

    if (size > unused) {
        if (size > (rb->mask + 1)) {
            size = rb->mask + 1;
        }

        rb->out += size - unused;
    }

    offset = rb->in & rb->mask;

    remain = rb->mask + 1 - offset;
    remain = remain > size ? size : remain;

    memcpy(((uint8_t *)(rb->pool)) + offset, data, remain);
    memcpy(rb->pool, (uint8_t *)data + remain, size - remain);

    rb->in += size;

    return size;
}

/*****************************************************************************
* @brief        预读环形缓冲区的数据，
*               多线程读取需加锁，
*               单读线程可无需锁
* 
* @param[in]    rb          环形缓冲区实例
* @param[in]    data        数据指针
* @param[in]    size        读取长度（字节）
* 
* @retval uint32_t          实际预读的字节数
*****************************************************************************/
uint32_t chry_ringbuffer_peek(chry_ringbuffer_t *rb, void *data, uint32_t size)
{
    uint32_t used;
    uint32_t offset;
    uint32_t remain;

    used = rb->in - rb->out;
    if (size > used) {
        size = used;
    }

    offset = rb->out & rb->mask;

    remain = rb->mask + 1 - offset;
    remain = remain > size ? size : remain;

    memcpy(data, ((uint8_t *)(rb->pool)) + offset, remain);
    memcpy((uint8_t *)data + remain, rb->pool, size - remain);

    return size;
}

/*****************************************************************************
* @brief        从环形缓冲区读取数据，
*               多线程读取需加锁，
*               单读线程可无需锁
* 
* @param[in]    rb          环形缓冲区实例
* @param[in]    data        数据指针
* @param[in]    size        读取长度（字节）
* 
* @retval uint32_t          实际读取的字节数
*****************************************************************************/
uint32_t chry_ringbuffer_read(chry_ringbuffer_t *rb, void *data, uint32_t size)
{
    size = chry_ringbuffer_peek(rb, data, size);
    rb->out += size;
    return size;
}

/*****************************************************************************
* @brief        丢弃环形缓冲区中的数据，
*               多线程读取需加锁，
*               单读线程可无需锁
* 
* @param[in]    rb          环形缓冲区实例
* @param[in]    size        丢弃长度（字节）
* 
* @retval uint32_t          实际丢弃的字节数
*****************************************************************************/
uint32_t chry_ringbuffer_drop(chry_ringbuffer_t *rb, uint32_t size)
{
    uint32_t used;

    used = rb->in - rb->out;
    if (size > used) {
        size = used;
    }

    rb->out += size;
    return size;
}

/*****************************************************************************
* @brief        配置线性写入，获取写指针与最大连续空间
*               
* @param[in]    rb          环形缓冲区实例
* @param[in]    size        保存最大连续空间（字节）的指针
* 
* @retval void*             可写入的内存指针
*****************************************************************************/
void *chry_ringbuffer_linear_write_setup(chry_ringbuffer_t *rb, uint32_t *size)
{
    uint32_t unused;
    uint32_t offset;
    uint32_t remain;

    unused = (rb->mask + 1) - (rb->in - rb->out);

    offset = rb->in & rb->mask;

    remain = rb->mask + 1 - offset;
    remain = remain > unused ? unused : remain;

    if (remain) {
        *size = remain;
        return ((uint8_t *)(rb->pool)) + offset;
    } else {
        *size = unused - remain;
        return rb->pool;
    }
}

/*****************************************************************************
* @brief        配置线性读取，获取读指针与最大连续空间
* 
* @param[in]    rb          环形缓冲区实例
* @param[in]    size        保存最大连续空间（字节）的指针
* 
* @retval void*             可读取的内存指针
*****************************************************************************/
void *chry_ringbuffer_linear_read_setup(chry_ringbuffer_t *rb, uint32_t *size)
{
    uint32_t used;
    uint32_t offset;
    uint32_t remain;

    used = rb->in - rb->out;

    offset = rb->out & rb->mask;

    remain = rb->mask + 1 - offset;
    remain = remain > used ? used : remain;

    if (remain) {
        *size = remain;
        return ((uint8_t *)(rb->pool)) + offset;
    } else {
        *size = used - remain;
        return rb->pool;
    }
}

/*****************************************************************************
* @brief        完成线性写入，仅推进写指针
* 
* @param[in]    rb          环形缓冲区实例
* @param[in]    size        写入长度（字节）
* 
* @retval uint32_t          实际写入的字节数
*****************************************************************************/
uint32_t chry_ringbuffer_linear_write_done(chry_ringbuffer_t *rb, uint32_t size)
{
    uint32_t unused;

    unused = (rb->mask + 1) - (rb->in - rb->out);
    if (size > unused) {
        size = unused;
    }
    rb->in += size;

    return size;
}

/*****************************************************************************
* @brief        完成线性读取，仅推进读指针
* 
* @param[in]    rb          环形缓冲区实例
* @param[in]    size        读取长度（字节）
* 
* @retval uint32_t          实际读取的字节数
*****************************************************************************/
uint32_t chry_ringbuffer_linear_read_done(chry_ringbuffer_t *rb, uint32_t size)
{
    return chry_ringbuffer_drop(rb, size);
}
