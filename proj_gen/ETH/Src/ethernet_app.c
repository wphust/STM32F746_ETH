/*
 * ethernet_app.c
 *
 *  Created on: 28 Oct 2017
 *      Author: wang_p
 */
#include "../Utilities/Log/lcd_log.h"
#include "ethernetif.h"

/**
 * this func is declared in ethernetif.h (with a weak linker attribute for overriding by user).
 */
void ethernetif_notify_conn_changed(struct netif *netif) {
	if(netif_is_link_up(netif))
	{
		LCD_UsrLog("ETH Connected.\n");
	}
	else
	{
		LCD_UsrLog("ETH Disconnected.\n");
	}
}

/**
 * Callback function delcared in netif.h.
 * void (*netif_status_callback_fn)(struct netif *netif);
 */
void ethernetif_status_callback(struct netif *netif)
{
	uint8_t iptxt[20];
	sprintf((char *) iptxt, "%s",
			ip4addr_ntoa((const ip4_addr_t *) &netif->ip_addr));
	LCD_UsrLog("ETH interface status changed. Current IP address: %s\n", iptxt);
}