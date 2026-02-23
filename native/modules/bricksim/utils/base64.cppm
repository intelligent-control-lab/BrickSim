export module bricksim.utils.base64;

import std;

namespace bricksim::base64 {

constexpr char padding_char{'='};

constexpr std::string_view alphabet =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

consteval auto make_decode_lut() {
	std::array<std::int16_t, 256> lut{};
	lut.fill(-1);
	for (int i = 0; i < 64; ++i) {
		lut[static_cast<unsigned char>(alphabet[static_cast<std::size_t>(i)])] =
		    static_cast<std::int16_t>(i);
	}
	// keep '=' as -2 if you ever want to treat it specially; we don't decode it
	lut[static_cast<unsigned char>(padding_char)] = -2;
	return lut;
}

constexpr auto decode_lut = make_decode_lut();

int decode6(unsigned char c) {
	return static_cast<int>(decode_lut[c]); // -1 invalid, 0..63 ok, -2 '='
}

export constexpr std::size_t encoded_size(std::size_t nbytes) noexcept {
	return ((nbytes + 2) / 3) * 4;
}

export std::size_t decoded_size(std::string_view b64) {
	if (b64.empty())
		return 0;
	if ((b64.size() & 3) != 0) {
		throw std::invalid_argument{"base64: size not divisible by 4"};
	}

	const std::size_t n = b64.size();
	std::size_t pad = 0;
	if (b64[n - 1] == padding_char) {
		pad = 1;
		if (n >= 2 && b64[n - 2] == padding_char)
			pad = 2;
	}

	// '=' must appear only at the end (if at all)
	if (auto pos = b64.find(padding_char); pos != std::string_view::npos) {
		if (pos != n - pad) {
			throw std::invalid_argument{"base64: invalid padding placement"};
		}
	}

	return (n / 4) * 3 - pad;
}

export std::size_t encode_into(std::span<const std::byte> in,
                               std::span<char> out) {
	const std::size_t n = in.size();
	const std::size_t need = encoded_size(n);
	if (out.size() < need) {
		throw std::length_error{"base64::encode_into: output too small"};
	}

	auto out_used = out.first(need);
	std::fill(out_used.begin(), out_used.end(), padding_char);

	const auto *bytes = reinterpret_cast<const std::uint8_t *>(in.data());
	char *curr = out_used.data();

	for (std::size_t i = n / 3; i; --i) {
		const std::uint8_t t1 = *bytes++;
		const std::uint8_t t2 = *bytes++;
		const std::uint8_t t3 = *bytes++;

		*curr++ = alphabet[t1 >> 2];
		*curr++ = alphabet[((t1 & 0x03u) << 4) | (t2 >> 4)];
		*curr++ = alphabet[((t2 & 0x0Fu) << 2) | (t3 >> 6)];
		*curr++ = alphabet[t3 & 0x3Fu];
	}

	switch (n % 3) {
	case 0:
		break;
	case 1: {
		const std::uint8_t t1 = bytes[0];
		*curr++ = alphabet[t1 >> 2];
		*curr++ = alphabet[(t1 & 0x03u) << 4];
		// remaining are '=' (already filled)
		break;
	}
	case 2: {
		const std::uint8_t t1 = bytes[0];
		const std::uint8_t t2 = bytes[1];
		*curr++ = alphabet[t1 >> 2];
		*curr++ = alphabet[((t1 & 0x03u) << 4) | (t2 >> 4)];
		*curr++ = alphabet[(t2 & 0x0Fu) << 2];
		// remaining is '=' (already filled)
		break;
	}
	default:
		throw std::runtime_error{"base64: unreachable"};
	}

	return need;
}

export std::string encode(std::span<const std::byte> in) {
	std::string out(encoded_size(in.size()), padding_char);
	(void)encode_into(in, std::span<char>{out.data(), out.size()});
	return out;
}

export std::size_t decode_into(std::string_view b64, std::span<std::byte> out) {
	if (b64.empty())
		return 0;

	const std::size_t need = decoded_size(b64);
	if (out.size() < need) {
		throw std::length_error{"base64::decode_into: output too small"};
	}

	const std::size_t n = b64.size();
	std::size_t pad = 0;
	if (b64[n - 1] == padding_char) {
		pad = 1;
		if (n >= 2 && b64[n - 2] == padding_char)
			pad = 2;
	}

	std::byte *curr = out.data();
	const unsigned char *p =
	    reinterpret_cast<const unsigned char *>(b64.data());

	auto emit3 = [&](int v1, int v2, int v3, int v4) {
		if ((v1 | v2 | v3 | v4) < 0) {
			throw std::invalid_argument{"base64: invalid character"};
		}
		const std::uint32_t x =
		    (std::uint32_t(v1) << 18) | (std::uint32_t(v2) << 12) |
		    (std::uint32_t(v3) << 6) | (std::uint32_t(v4) << 0);
		*curr++ = std::byte((x >> 16) & 0xFFu);
		*curr++ = std::byte((x >> 8) & 0xFFu);
		*curr++ = std::byte((x >> 0) & 0xFFu);
	};

	// full quads except the last one if padded
	for (std::size_t i = (n / 4) - (pad != 0); i; --i) {
		const int v1 = decode6(*p++);
		const int v2 = decode6(*p++);
		const int v3 = decode6(*p++);
		const int v4 = decode6(*p++);
		emit3(v1, v2, v3, v4);
	}

	if (pad == 1) {
		const int v1 = decode6(*p++);
		const int v2 = decode6(*p++);
		const int v3 = decode6(*p++);
		if ((v1 | v2 | v3) < 0)
			throw std::invalid_argument{"base64: invalid character"};

		const std::uint32_t x = (std::uint32_t(v1) << 18) |
		                        (std::uint32_t(v2) << 12) |
		                        (std::uint32_t(v3) << 6);

		*curr++ = std::byte((x >> 16) & 0xFFu);
		*curr++ = std::byte((x >> 8) & 0xFFu);
	} else if (pad == 2) {
		const int v1 = decode6(*p++);
		const int v2 = decode6(*p++);
		if ((v1 | v2) < 0)
			throw std::invalid_argument{"base64: invalid character"};

		const std::uint32_t x =
		    (std::uint32_t(v1) << 18) | (std::uint32_t(v2) << 12);

		*curr++ = std::byte((x >> 16) & 0xFFu);
	}

	return need;
}

export std::vector<std::byte> decode(std::string_view b64) {
	const std::size_t need = decoded_size(b64);
	std::vector<std::byte> out(need);
	(void)decode_into(b64, out);
	return out;
}

} // namespace bricksim::base64
