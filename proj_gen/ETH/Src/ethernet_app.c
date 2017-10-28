/*
 * ethernet_app.c
 *
 *  Created on: 28 Oct 2017
 *      Author: wang_p
 */
#include "../Utilities/Log/lcd_log.h"
#include "ethernetif.h"


void ethernetif_notify_conn_changed(struct netif *netif) {
	if(netif_is_link_up(netif))
	{
//		uint8_t iptxt[20];
//		sprintf((char *) iptxt, "%s",
//				ip4addr_ntoa((const ip4_addr_t *) &netif->ip_addr));
//		LCD_UsrLog("netif UP. IP address: %s\n", iptxt);
		LCD_UsrLog("ETH Up. \n");
	}
	else
	{
		LCD_UsrLog("ETH Down. \n");
	}
}
