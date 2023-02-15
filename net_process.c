#include "ethernet.h"
#include <c6x.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

///////////////////////////////////////
#ifndef _BSDTYPES_DEFINED
/* also defined in mingw/gmon.h and in w32api/winsock[2].h */
#ifndef __u_char_defined
typedef unsigned char u_char;
#define __u_char_defined
#endif
#ifndef __u_short_defined
typedef unsigned short u_short;
#define __u_short_defined
#endif
#ifndef __u_int_defined
typedef unsigned int u_int;
#define __u_int_defined
#endif
#ifndef __u_long_defined
typedef unsigned long u_long;
#define __u_long_defined
#endif
#define _BSDTYPES_DEFINED
#endif

#define __packed __attribute__((packed))
#define __aligned(x) __attribute__((aligned(x)))

/*
 * Define constants based on RFC 883, RFC 1034, RFC 1035
 */
#define NS_PACKETSZ 512 /* maximum packet size */
#define NS_MAXDNAME 1025 /* maximum domain name */
#define NS_MAXCDNAME 255 /* maximum compressed domain name */
#define NS_MAXLABEL 63 /* maximum length of domain label */
#define NS_HFIXEDSZ 12 /* #/bytes of fixed data in header */
#define NS_QFIXEDSZ 4 /* #/bytes of fixed data in query */
#define NS_RRFIXEDSZ 10 /* #/bytes of fixed data in r record */
#define NS_INT32SZ 4 /* #/bytes of data in a u_int32_t */
#define NS_INT16SZ 2 /* #/bytes of data in a u_int16_t */
#define NS_INT8SZ 1 /* #/bytes of data in a u_int8_t */
#define NS_INADDRSZ 4 /* IPv4 T_A */
#define NS_IN6ADDRSZ 16 /* IPv6 T_AAAA */
#define NS_CMPRSFLGS 0xc0 /* Flag bits indicating name compression. */
#define NS_DEFAULTPORT 53 /* For both TCP and UDP. */

/* Protocols common to RFC 1700, POSIX, and X/Open. */
#define IPPROTO_IP 0 /* dummy for IP */
#define IPPROTO_ICMP 1 /* control message protocol */
#define IPPROTO_TCP 6 /* tcp */
#define IPPROTO_UDP 17 /* user datagram protocol */

///////////////////////////////////////

typedef uint32_t in_addr_t;
typedef char* caddr_t;

#define CPU_swap_u16(value) \
    (((value & 0xff) << 8) | ((value >> 8) & 0xff))

static inline uint32_t CPU_swap_u32(uint32_t value)
{
    uint32_t byte1, byte2, byte3, byte4, swapped;

    byte4 = (value >> 24) & 0xff;
    byte3 = (value >> 16) & 0xff;
    byte2 = (value >> 8) & 0xff;
    byte1 = value & 0xff;

    swapped = (byte1 << 24) | (byte2 << 16) | (byte3 << 8) | byte4;
    return swapped;
}

static inline uint32_t ntohl(uint32_t _x)
{
    return CPU_swap_u32(_x);
}

static inline uint16_t ntohs(uint16_t _x)
{
    return CPU_swap_u16(_x);
}

static inline uint32_t htonl(uint32_t _x)
{
    return CPU_swap_u32(_x);
}

/*
 * Structure of a 10Mb/s Ethernet header.
 */
struct ether_header {
    u_char ether_dhost[ETHER_ADDR_LEN];
    u_char ether_shost[ETHER_ADDR_LEN];
    u_short ether_type;
} __packed __aligned(1);

struct __attribute__((__packed__)) in_addr {
    in_addr_t s_addr;
} __packed __aligned(1);

/*
 * Definitions for internet protocol version 4.
 *
 * Per RFC 791, September 1981.
 */
#define IPVERSION 4

/*
 * Structure of an internet header, naked of options.
 */
struct __attribute__((__packed__)) ip {
    u_char ip_vhl; /* header length */ /* version */
    u_char ip_tos; /* type of service */
    u_short ip_len; /* total length */
    u_short ip_id; /* identification */
    u_short ip_off; /* fragment offset field */
#define IP_RF 0x8000 /* reserved fragment flag */
#define IP_DF 0x4000 /* dont fragment flag */
#define IP_MF 0x2000 /* more fragments flag */
#define IP_OFFMASK 0x1fff /* mask for fragmenting bits */
    u_char ip_ttl; /* time to live */
    u_char ip_p; /* protocol */
    u_short ip_sum; /* checksum */
    struct in_addr ip_src, ip_dst; /* source and dest address */
} __packed __aligned(1);

typedef unsigned int __uint32_t;
typedef __uint32_t u_int32_t;
typedef u_int32_t tcp_seq;

#define tcp6_seq tcp_seq /* for KAME src sync over BSD*'s */
#define tcp6hdr tcphdr /* for KAME src sync over BSD*'s */

/*
 * TCP header.
 * Per RFC 793, September, 1981.
 */
struct __attribute__((__packed__)) tcphdr {
    u_short th_sport; /* source port */
    u_short th_dport; /* destination port */
    tcp_seq th_seq; /* sequence number */
    tcp_seq th_ack; /* acknowledgement number */
    u_char th_x2_off; /* (unused) */ /* data offset */
    u_char th_flags;
    u_short th_win; /* window */
    u_short th_sum; /* checksum */
    u_short th_urp; /* urgent pointer */
} __packed __aligned(1);

/*
 * UDP protocol header.
 * Per RFC 768, September, 1981.
 */
struct __attribute__((__packed__)) udphdr {
    u_short uh_sport; /* source port */
    u_short uh_dport; /* destination port */
    u_short uh_ulen; /* udp length */
    u_short uh_sum; /* udp checksum */
} __packed __aligned(1);
//////////////////////////////////////////////////////////
/*
 * Overlay for ip header used by other protocols (tcp, udp).
 */
struct __attribute__((__packed__)) ipovly {
    caddr_t ih_next;
    caddr_t ih_prev; /* for protocol sequence q's */
    u_char ih_x1; /* (unused) */
    u_char ih_pr; /* protocol */
    u_short ih_len; /* protocol length */
    struct in_addr ih_src; /* source internet address */
    struct in_addr ih_dst; /* destination internet address */
} __packed __aligned(1);

struct __attribute__((__packed__)) tcpiphdr {
    struct ipovly ti_i; /* overlaid ip structure */
    struct tcphdr ti_t; /* tcp header */
} __packed __aligned(1);

#define ti_next ti_i.ih_next
#define ti_prev ti_i.ih_prev
#define ti_x1 ti_i.ih_x1
#define ti_pr ti_i.ih_pr
#define ti_len ti_i.ih_len
#define ti_src ti_i.ih_src
#define ti_dst ti_i.ih_dst
#define ti_sport ti_t.th_sport
#define ti_dport ti_t.th_dport
#define ti_seq ti_t.th_seq
#define ti_ack ti_t.th_ack
#define ti_x2 ti_t.th_x2
#define ti_off ti_t.th_off
#define ti_flags ti_t.th_flags
#define ti_win ti_t.th_win
#define ti_sum ti_t.th_sum
#define ti_urp ti_t.th_urp

/*
 * Process a received Ethernet packet;
 * the packet is in the mbuf chain m without
 * the ether header, which is provided separately.
 */
u_short
_ether_input(u_char* frame)
{
    struct ether_header* eh = (struct ether_header*)frame;

    printf("Layer 1 : ");
    printf("Ethernet Address Source: ");
    int i = ETHER_ADDR_LEN;

    u_char* cp = eh->ether_shost;
    do {
        i--;
        printf("%02X%c", *cp++, i ? ':' : '\t');
    } while (i > 0);

    printf("Dest: ");
    i = ETHER_ADDR_LEN;

    cp = eh->ether_dhost;
    do {
        i--;
        printf("%02X%c", *cp++, i ? ':' : '\n');
    } while (i > 0);

    return ntohs(eh->ether_type);
}

/*
 * Standard type definitions.
 */
typedef uint8_t sa_family_t;
typedef uint32_t socklen_t;

#define SPRINTF(x) ((size_t)sprintf x)

#define AF_INET 2 /* internetwork: UDP, TCP, etc. */
#define AF_INET6 28 /* IPv6 */
#if 1
/* const char *
 * inet_ntop4(src, dst, size)
 *	format an IPv4 address, more or less like inet_ntoa()
 * return:
 *	`dst' (as a const)
 * notes:
 *	(1) uses no statics
 *	(2) takes a u_char* not an in_addr as input
 * author:
 *	Paul Vixie, 1996.
 */
static const char*
inet_ntop4(
    const u_char* src,
    char* dst,
    socklen_t size)
{
    static const char fmt[] = "%u.%u.%u.%u";
    char tmp[sizeof "255.255.255.255"];

    if (SPRINTF((tmp, fmt, src[0], src[1], src[2], src[3])) > size) {
        return (NULL);
    }
    strcpy(dst, tmp);
    return (dst);
}

/*
 * Convert network-format internet address
 * to base 256 d.d.d.d representation.
 */
static char*
inet_ntoa(
    struct in_addr in)
{
    static char ret[18];

    strcpy(ret, "[inet_ntoa error]");
    (void)inet_ntop4(&in, ret, sizeof ret);
    return (ret);
}
#endif
/*
 * Ip input routine.  Checksum and byte swap header.  If fragmented
 * try to reassemble.  Process options.  Pass to next level.
 */
void _ip_input(u_char* packet)
{
    struct ip* ip = (struct ip*)packet;

    char buf[4 * sizeof "123"];

    struct tcpiphdr* ti;
    struct udphdr* uh;

    /*
     * Convert fields to host representation.
     */
    ip->ip_len = ntohs(ip->ip_len);
    ntohs(ip->ip_id);
    ip->ip_off = ntohs(ip->ip_off);

    printf("Layer 2 : ");
    printf("IP Source:  %s\t\t", inet_ntoa(ip->ip_src));
    printf("Dest: %s\n", inet_ntoa(ip->ip_dst));
    if (IP_MF == ip->ip_off) {
        puts("More Fragments");
    }

    printf("Layer 3 : ");
    switch (ip->ip_p) {

    case IPPROTO_TCP:

        ti = (struct tcpiphdr*)ip;

        strcpy(buf, inet_ntoa(ti->ti_dst));
        printf("TCP Source: %s:%d\tDest: %s:%d\n", inet_ntoa(ti->ti_src), ntohs(ti->ti_sport),
            buf, ntohs(ti->ti_dport));

        break;

    case IPPROTO_UDP:

        uh = (struct udphdr*)((caddr_t)ip + sizeof(struct ip));

        strcpy(buf, inet_ntoa(ip->ip_dst));
        printf("UDP Source: %s:%d\tDest: %s:%d\n", inet_ntoa(ip->ip_src), ntohs(uh->uh_sport),
            buf, ntohs(uh->uh_dport));
        break;
    }

    puts("");
}

void stack_main(char* pBuffer)
{
    u_short ether_type = _ether_input(pBuffer);

    switch (ether_type) {

    case ETHERTYPE_IP:
        _ip_input(pBuffer + sizeof(struct ether_header));

        break;

    case ETHERTYPE_ARP:
        break;
    }
}
