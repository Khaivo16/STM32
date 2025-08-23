#include "max30100.h"
#include "ring_buffer.h"

static rb_data_type  ring_buffer[RINGBUFFER_SIZE] ;
uint16_t             buf_size = RINGBUFFER_SIZE ;
uint16_t             buf_count = 0 ;
static uint16_t      r_index = 0 ;
static uint16_t      w_index = 0 ;


void rb_init(void) 
{
    r_index = 0 ;
    w_index = 0 ;
    buf_count = 0 ;
}

void rb_destruct(void)
{
    r_index = 0 ;
    w_index = 0 ;
    buf_count = 0 ;
}

/**
 * Add an element to the beginning of the buffer:
 * @returns: 0 if the addition caused overwriting an existing element.
 */
bool                rb_unshift(rb_data_type value) 
{
    bool result = 0 ;
    
    r_index = (r_index + buf_size - 1) % buf_size ;

    ring_buffer[r_index] = value ;
    if (buf_count == buf_size) {
        w_index = (w_index + buf_size - 1) % buf_size ;
        result = 0 ;
    } else {
        if (buf_count++ == 0) {
            w_index = r_index ;
        }
        result = 1 ;
    }
    return( result ) ;
}

/**
 * Add an element to the end of the buffer:
 * @returns: 0 if the addition caused overwriting an existing element.
 */
bool                rb_push(rb_data_type value) 
{
    bool result = 1 ;

    w_index = (w_index + 1) % buf_size ;
    if (w_index == r_index) {
        result = 0 ;
    }

    ring_buffer[w_index] = value ;
    buf_count++ ;
    
    return( result ) ;
}

/**
 * Removes an element from the beginning of the buffer.
*/
rb_data_type        rb_shift(void) 
{
    rb_data_type result ;
    result = ring_buffer[r_index] ;
    r_index = (r_index + 1) % buf_size ;
    buf_count-- ;
    return( result ) ;
}

/**
 * Removes an element from the end of the buffer.
 */
rb_data_type        rb_pop(void) 
{
    rb_data_type result ;
    result = ring_buffer[w_index] ;
    w_index = (w_index + buf_size - 1) % buf_size ;
    buf_count-- ;
    return( result ) ;
}

/**
 * Returns the element of the beginnig of the buffer.
 */
rb_data_type  rb_first(void) 
{
    return( ring_buffer[r_index] ) ;
}

/**
 * Returns the element of the end of the buffer.
 */
rb_data_type  rb_last(void) 
{
    return( ring_buffer[w_index] ) ;
}

/**
 * In C++ [] operator
 */
rb_data_type        rb_data_at(uint16_t index) 
{
    uint16_t buf_index ;
    buf_index = (r_index + index) % buf_size ;
    return( ring_buffer[ buf_index ] ) ;
}

/**
 * Resets the buffer to a clean status, droppig any reference to current elements
 * and making all buffer position available again.
 */
void          rb_clear(void) 
{
    r_index = 0 ;
    w_index = 0 ;
    buf_count = 0 ;
}
/* [] END OF FILE */
