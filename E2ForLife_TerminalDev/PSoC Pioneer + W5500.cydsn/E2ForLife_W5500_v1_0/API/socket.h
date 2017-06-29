/**
 * \addtogroup e2forlife_w5500
 * @{
 */
/**
 * Copyright (c) 2015, E2ForLife.com
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of E2ForLife.com nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \file socket.h
 * \author Chuck Erhardt (chuck@e2forlife.com)
 * 
 * Changes: Carlos Diaz (carlos.santiago.diaz@gmail.com) 2017
 * Header file for the socket functions.
 */

#ifndef `$INSTANCE_NAME`_SOCKET_H
#define `$INSTANCE_NAME`_SOCKET_H

#include <cytypes.h>
#include <CyLib.h>
    
cystatus `$INSTANCE_NAME`_ExecuteSocketCommand(uint8_t socket, uint8_t cmd );
uint8_t `$INSTANCE_NAME`_SocketOpen( uint16_t port, uint8_t flags );
cystatus `$INSTANCE_NAME`_SocketClose( uint8_t sock, uint8_t discon );
cystatus `$INSTANCE_NAME`_SocketDisconnect( uint8_t sock );
cystatus `$INSTANCE_NAME`_SocketSendComplete( uint8_t socket );

#endif /* `$INSTANCE_NAME`_SOCKET_H */

/* [] END OF FILE */
