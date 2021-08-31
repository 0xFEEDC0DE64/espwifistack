#pragma once

// system includes
#include <stdint.h>
#include <string>
#include <string_view>
#include <array>

// esp-idf includes
#include <esp_wifi_types.h>
#include <esp_netif_ip_addr.h>
#include <esp_netif_types.h>
#include <lwip/ip_addr.h>

// 3rdparty lib includes
#include <tl/expected.hpp>

namespace wifi_stack {
bool wifi_ap_config_equal(const wifi_ap_config_t& lhs, const wifi_ap_config_t& rhs);
bool wifi_sta_config_equal(const wifi_sta_config_t& lhs, const wifi_sta_config_t& rhs);

std::string toString(wifi_auth_mode_t authMode);
std::string toString(wifi_cipher_type_t cipherType);
std::string toString(esp_interface_t interface);
std::string toString(esp_netif_dhcp_status_t status);

// A class to make it easier to handle and pass around mac addresses / bssids

class mac_t : public std::array<uint8_t, 6>
{
public:
    using std::array<uint8_t, 6>::array;

    constexpr explicit mac_t(const uint8_t (&arr)[6]) noexcept
    {
        std::copy(std::begin(arr), std::end(arr), std::begin(*this));
    }

    void copyTo(uint8_t (&arr)[6]) const noexcept
    {
        std::copy(std::begin(*this), std::end(*this), std::begin(arr));
    }
};

std::string toString(const mac_t &mac);

// A class to make it easier to handle and pass around IP addresses
// Implementation taken from arduino-esp32

class ip_address_t
{
public:
    using value_t = uint32_t;

private:
    union {
        std::array<uint8_t, 4> _bytes;  // IPv4 address
        value_t _value;
        static_assert(sizeof(_bytes) == sizeof(_value));
    };

public:
    // Constructors
    constexpr ip_address_t() noexcept : _value{0} {}
    constexpr ip_address_t(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) noexcept : _bytes{b0, b1, b2, b3} {}
    constexpr explicit ip_address_t(value_t address) noexcept : _value{address} {}
    constexpr explicit ip_address_t(ip4_addr_t address) noexcept : _value{address.addr} {}
    constexpr explicit ip_address_t(esp_ip4_addr_t address) noexcept : _value{address.addr} {}
    constexpr ip_address_t(std::array<uint8_t, 4> bytes) noexcept : _bytes{bytes} {}

    static tl::expected<ip_address_t, std::string> parseFromString(std::string_view address);

    // Access the raw byte array containing the address.  Because this returns a pointer
    // to the internal structure rather than a copy of the address this function should only
    // be used when you know that the usage of the returned uint8_t* will be transient and not
    // stored.
    constexpr std::array<uint8_t, 4> &bytes() noexcept { return _bytes; }
    constexpr std::array<uint8_t, 4> bytes() const noexcept { return _bytes; }

    constexpr value_t value() const noexcept { return _value; }

    friend constexpr bool operator==(const ip_address_t &left, const ip_address_t &right) noexcept { return left._value == right._value; }
    friend constexpr bool operator!=(const ip_address_t &left, const ip_address_t &right) noexcept { return !(left == right); }

    // Overloaded index operator to allow getting and setting individual octets of the address
    constexpr uint8_t& operator[](int index) noexcept { return _bytes[index]; }
    constexpr uint8_t operator[](int index) const noexcept { return _bytes[index]; }

    // Overloaded copy operators to allow initialisation of ip_address_t objects from other types
    constexpr ip_address_t& operator=(const ip_address_t &other) noexcept { _value = other._value; return *this; }
    constexpr ip_address_t& operator=(std::array<uint8_t, 4> bytes) noexcept { _bytes = bytes; return *this; }
    constexpr ip_address_t& operator=(value_t value) noexcept { _value = value; return *this; }
};

std::string toString(const ip_address_t &val);

ip_address_t wifi_calculate_network_id(ip_address_t ip, ip_address_t subnet);
ip_address_t wifi_calculate_broadcast(ip_address_t ip, ip_address_t subnet);
uint8_t wifi_calculate_subnet_cidr(ip_address_t subnetMask);

std::string toString(ip4_addr_t val);
std::string toString(const ip6_addr_t &val);
std::string toString(ip_addr_t val);

inline std::string toString(const esp_ip4_addr_t &val)
{ return toString(*reinterpret_cast<const ip4_addr_t *>(&val)); }
inline std::string toString(const esp_ip6_addr_t &val)
{ return toString(*reinterpret_cast<const ip6_addr_t *>(&val)); }

} // namespace wifi_stack
