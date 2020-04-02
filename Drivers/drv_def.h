/*
*********************************************************************************************************
*                                              uC/CAN
*                                      The Embedded CAN suite
*
*                 Copyright by Embedded Office GmbH & Co. KG www.embedded-office.com
*                    Copyright 1992-2020 Silicon Laboratories Inc. www.silabs.com
*
*                                 SPDX-License-Identifier: APACHE-2.0
*
*               This software is subject to an open source license and is distributed by
*                Silicon Laboratories Inc. pursuant to the terms of the Apache License,
*                    Version 2.0 available at www.apache.org/licenses/LICENSE-2.0.
*
*********************************************************************************************************
*/


/*
****************************************************************************************************
* Filename : drv_def.h
* Version  : V2.42.01
****************************************************************************************************
*/

#ifndef _DRV_DEF_H
#define _DRV_DEF_H


/**************************************************************************************************/
/*                         ACCESS TYPE DEFINITIONS                                                */
/**************************************************************************************************/


/*------------------------------------------------------------------------------------------------*/
/*! \brief                 EXLCUSIVE ACCESS
*
*            This define holds the value for mode coding bit 7: exclusive access
*/
/*------------------------------------------------------------------------------------------------*/
#define DEV_EXCLUSIVE  (0u << 7u)

/*------------------------------------------------------------------------------------------------*/
/*! \brief                 SHARED ACCESS
*
*            This define holds the value for mode coding bit 7: shared access
*/
/*------------------------------------------------------------------------------------------------*/
#define DEV_SHARE      (1u << 7u)


/**************************************************************************************************/
/*                         PERMISSION DEFINITIONS                                                 */
/**************************************************************************************************/


/*------------------------------------------------------------------------------------------------*/
/*! \brief                 EXECUTE PERMISSION
*
*            This define holds the value for mode coding bit 0: execute permission enabled
*/
/*------------------------------------------------------------------------------------------------*/
#define DEV_EXE        (1u << 0u)

/*------------------------------------------------------------------------------------------------*/
/*! \brief                 WRITE PERMISSION
*
*            This define holds the value for mode coding bit 1: write permission enabled
*/
/*------------------------------------------------------------------------------------------------*/
#define DEV_WO         (1u << 1u)

/*------------------------------------------------------------------------------------------------*/
/*! \brief                 READ PERMISSION
*
*            This define holds the value for mode coding bit 2: read permission enabled
*/
/*------------------------------------------------------------------------------------------------*/
#define DEV_RO         (1u << 2u)


/**************************************************************************************************/
/*                         ACCESS MODE DEFINITIONS                                                */
/**************************************************************************************************/


/*------------------------------------------------------------------------------------------------*/
/*! \brief                 EXCLUSIVE READ/WRITE ACCESS
*
*            This define holds the value for mode: exclusive read/write access
*/
/*------------------------------------------------------------------------------------------------*/
#define DEV_RW         (DEV_EXCLUSIVE | DEV_WO | DEV_RO)

/*------------------------------------------------------------------------------------------------*/
/*! \brief                 EXCLUSIVE READ/WRITE/EXECUTE ACCESS
*
*            This define holds the value for mode: exclusive read/write/execute access
*/
/*------------------------------------------------------------------------------------------------*/
#define DEV_RWX        (DEV_EXCLUSIVE | DEV_WO | DEV_RO | DEV_EXE)

/*------------------------------------------------------------------------------------------------*/
/*! \brief                 SHARED WRITE ACCESS
*
*            This define holds the value for mode: shared write access
*/
/*------------------------------------------------------------------------------------------------*/
#define DEV_SHWO       (DEV_SHARE | DEV_WO)

/*------------------------------------------------------------------------------------------------*/
/*! \brief                 SHARED READ ACCESS
*
*            This define holds the value for mode: shared read access
*/
/*------------------------------------------------------------------------------------------------*/
#define DEV_SHRO       (DEV_SHARE | DEV_RO)

/*------------------------------------------------------------------------------------------------*/
/*! \brief                 SHARED READ/WRITE ACCESS
*
*            This define holds the value for mode: shared read/write access
*/
/*------------------------------------------------------------------------------------------------*/
#define DEV_SHRW       (DEV_SHARE | DEV_WO | DEV_RO)

/*------------------------------------------------------------------------------------------------*/
/*! \brief                 SHARED READ/WRITE/EXECUTE ACCESS
*
*            This define holds the value for mode: shared read/write/execute access
*/
/*------------------------------------------------------------------------------------------------*/
#define DEV_SHRWX      (DEV_SHARE | DEV_WO | DEV_RO | DEV_EXE)


#endif  /* #ifndef _DRV_DEF_H */

