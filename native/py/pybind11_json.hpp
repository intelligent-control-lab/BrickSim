// Original: https://github.com/pybind/pybind11_json/blob/d0bf434be9d287d73a963ff28745542daf02c08f/include/pybind11_json/pybind11_json.hpp
// Modified by: Haowei Wen <haoweiwen@cmu.edu>

/***************************************************************************
* Copyright (c) 2019, Martin Renou                                         *
*                                                                          *
* Distributed under the terms of the BSD 3-Clause License.                 *
*                                                                          *
* The full license is in the file LICENSE, distributed with this software. *
****************************************************************************/

#pragma once

import bricksim.vendor;

#include <set>
#include <string>

#include <pybind11/pybind11.h>

namespace pyjson {
inline pybind11::object from_json(const nlohmann::json &j) {
	if (j.is_null()) {
		return pybind11::none();
	} else if (j.is_boolean()) {
		return pybind11::bool_(j.get<bool>());
	} else if (j.is_number_unsigned()) {
		return pybind11::int_(j.get<nlohmann::json::number_unsigned_t>());
	} else if (j.is_number_integer()) {
		return pybind11::int_(j.get<nlohmann::json::number_integer_t>());
	} else if (j.is_number_float()) {
		return pybind11::float_(j.get<double>());
	} else if (j.is_string()) {
		return pybind11::str(j.get<std::string>());
	} else if (j.is_array()) {
		pybind11::list obj(j.size());
		for (std::size_t i = 0; i < j.size(); i++) {
			obj[i] = from_json(j[i]);
		}
		return obj;
	} else // Object
	{
		pybind11::dict obj;
		for (nlohmann::json::const_iterator it = j.cbegin(); it != j.cend();
		     ++it) {
			obj[pybind11::str(it.key())] = from_json(it.value());
		}
		return obj;
	}
}

// --- NEW: nlohmann::ordered_json -> pybind11::object ------------------
inline pybind11::object from_json(const nlohmann::ordered_json &j) {
	if (j.is_null()) {
		return pybind11::none();
	} else if (j.is_boolean()) {
		return pybind11::bool_(j.get<bool>());
	} else if (j.is_number_unsigned()) {
		return pybind11::int_(
		    j.get<nlohmann::ordered_json::number_unsigned_t>());
	} else if (j.is_number_integer()) {
		return pybind11::int_(
		    j.get<nlohmann::ordered_json::number_integer_t>());
	} else if (j.is_number_float()) {
		return pybind11::float_(j.get<double>());
	} else if (j.is_string()) {
		return pybind11::str(j.get<std::string>());
	} else if (j.is_array()) {
		pybind11::list obj(j.size());
		for (std::size_t i = 0; i < j.size(); ++i) {
			obj[i] = from_json(j[i]); // uses this overload (ordered_json)
		}
		return obj;
	} else { // Object
		pybind11::dict obj;
		for (nlohmann::ordered_json::const_iterator it = j.cbegin();
		     it != j.cend(); ++it) {
			obj[pybind11::str(it.key())] = from_json(it.value());
		}
		return obj;
	}
}

inline nlohmann::json to_json(const pybind11::handle &obj,
                              std::set<const PyObject *> &refs) {
	if (obj.ptr() == nullptr || obj.is_none()) {
		return nullptr;
	}
	if (pybind11::isinstance<pybind11::bool_>(obj)) {
		return obj.cast<bool>();
	}
	if (pybind11::isinstance<pybind11::int_>(obj)) {
		try {
			nlohmann::json::number_integer_t s =
			    obj.cast<nlohmann::json::number_integer_t>();
			if (pybind11::int_(s).equal(obj)) {
				return s;
			}
		} catch (...) {
		}
		try {
			nlohmann::json::number_unsigned_t u =
			    obj.cast<nlohmann::json::number_unsigned_t>();
			if (pybind11::int_(u).equal(obj)) {
				return u;
			}
		} catch (...) {
		}
		throw std::runtime_error("to_json received an integer out of range for "
		                         "both nlohmann::json::number_integer_t and "
		                         "nlohmann::json::number_unsigned_t type: " +
		                         pybind11::repr(obj).cast<std::string>());
	}
	if (pybind11::isinstance<pybind11::float_>(obj)) {
		return obj.cast<double>();
	}
	if (pybind11::isinstance<pybind11::bytes>(obj)) {
		pybind11::module base64 = pybind11::module::import("base64");
		return base64.attr("b64encode")(obj)
		    .attr("decode")("utf-8")
		    .cast<std::string>();
	}
	if (pybind11::isinstance<pybind11::str>(obj)) {
		return obj.cast<std::string>();
	}
	if (pybind11::isinstance<pybind11::tuple>(obj) ||
	    pybind11::isinstance<pybind11::list>(obj)) {
		auto insert_ret = refs.insert(obj.ptr());
		if (!insert_ret.second) {
			throw std::runtime_error("Circular reference detected");
		}

		auto out = nlohmann::json::array();
		for (const pybind11::handle value : obj) {
			out.push_back(to_json(value, refs));
		}

		refs.erase(insert_ret.first);

		return out;
	}
	if (pybind11::isinstance<pybind11::dict>(obj)) {
		auto insert_ret = refs.insert(obj.ptr());
		if (!insert_ret.second) {
			throw std::runtime_error("Circular reference detected");
		}

		auto out = nlohmann::json::object();
		for (const pybind11::handle key : obj) {
			out[pybind11::str(key).cast<std::string>()] =
			    to_json(obj[key], refs);
		}

		refs.erase(insert_ret.first);

		return out;
	}

	throw std::runtime_error(
	    "to_json not implemented for this type of object: " +
	    pybind11::repr(obj).cast<std::string>());
}

inline nlohmann::json to_json(const pybind11::handle &obj) {
	std::set<const PyObject *> refs;
	return to_json(obj, refs);
}

// --- NEW: pybind11::handle -> nlohmann::ordered_json ------------------
inline nlohmann::ordered_json
to_ordered_json(const pybind11::handle &obj, std::set<const PyObject *> &refs) {
	if (obj.ptr() == nullptr || obj.is_none()) {
		return nullptr;
	}
	if (pybind11::isinstance<pybind11::bool_>(obj)) {
		return obj.cast<bool>();
	}
	if (pybind11::isinstance<pybind11::int_>(obj)) {
		try {
			nlohmann::ordered_json::number_integer_t s =
			    obj.cast<nlohmann::ordered_json::number_integer_t>();
			if (pybind11::int_(s).equal(obj)) {
				return s;
			}
		} catch (...) {
		}
		try {
			nlohmann::ordered_json::number_unsigned_t u =
			    obj.cast<nlohmann::ordered_json::number_unsigned_t>();
			if (pybind11::int_(u).equal(obj)) {
				return u;
			}
		} catch (...) {
		}
		throw std::runtime_error(
		    "to_json received an integer out of range for both "
		    "nlohmann::ordered_json::number_integer_t and "
		    "nlohmann::ordered_json::number_unsigned_t type: " +
		    pybind11::repr(obj).cast<std::string>());
	}
	if (pybind11::isinstance<pybind11::float_>(obj)) {
		return obj.cast<double>();
	}
	if (pybind11::isinstance<pybind11::bytes>(obj)) {
		pybind11::module base64 = pybind11::module::import("base64");
		return base64.attr("b64encode")(obj)
		    .attr("decode")("utf-8")
		    .cast<std::string>();
	}
	if (pybind11::isinstance<pybind11::str>(obj)) {
		return obj.cast<std::string>();
	}

	if (pybind11::isinstance<pybind11::tuple>(obj) ||
	    pybind11::isinstance<pybind11::list>(obj)) {
		auto insert_ret = refs.insert(obj.ptr());
		if (!insert_ret.second) {
			throw std::runtime_error("Circular reference detected");
		}

		auto out = nlohmann::ordered_json::array();
		for (const pybind11::handle value : obj) {
			out.push_back(to_ordered_json(value, refs));
		}

		refs.erase(insert_ret.first);
		return out;
	}

	if (pybind11::isinstance<pybind11::dict>(obj)) {
		auto insert_ret = refs.insert(obj.ptr());
		if (!insert_ret.second) {
			throw std::runtime_error("Circular reference detected");
		}

		auto out = nlohmann::ordered_json::object();
		for (const pybind11::handle key : obj) {
			out[pybind11::str(key).cast<std::string>()] =
			    to_ordered_json(obj[key], refs);
		}

		refs.erase(insert_ret.first);
		return out;
	}

	throw std::runtime_error(
	    "to_json not implemented for this type of object: " +
	    pybind11::repr(obj).cast<std::string>());
}

inline nlohmann::ordered_json to_ordered_json(const pybind11::handle &obj) {
	std::set<const PyObject *> refs;
	return to_ordered_json(obj, refs);
}

} // namespace pyjson

// nlohmann_json serializers
namespace nlohmann {
#define MAKE_NLJSON_SERIALIZER_DESERIALIZER(T)                                 \
	template <> struct adl_serializer<T> {                                     \
		inline static void to_json(json &j, const T &obj) {                    \
			j = pyjson::to_json(obj);                                          \
		}                                                                      \
                                                                               \
		inline static void to_json(ordered_json &j, const T &obj) {            \
			j = pyjson::to_ordered_json(obj);                                  \
		}                                                                      \
                                                                               \
		inline static T from_json(const json &j) {                             \
			return pyjson::from_json(j);                                       \
		}                                                                      \
                                                                               \
		inline static T from_json(const ordered_json &j) {                     \
			return pyjson::from_json(j);                                       \
		}                                                                      \
	}

#define MAKE_NLJSON_SERIALIZER_ONLY(T)                                         \
	template <> struct adl_serializer<T> {                                     \
		inline static void to_json(json &j, const T &obj) {                    \
			j = pyjson::to_json(obj);                                          \
		}                                                                      \
                                                                               \
		inline static void to_json(ordered_json &j, const T &obj) {            \
			j = pyjson::to_ordered_json(obj);                                  \
		}                                                                      \
	}

MAKE_NLJSON_SERIALIZER_DESERIALIZER(pybind11::object);

MAKE_NLJSON_SERIALIZER_DESERIALIZER(pybind11::bool_);
MAKE_NLJSON_SERIALIZER_DESERIALIZER(pybind11::int_);
MAKE_NLJSON_SERIALIZER_DESERIALIZER(pybind11::float_);
MAKE_NLJSON_SERIALIZER_DESERIALIZER(pybind11::str);

MAKE_NLJSON_SERIALIZER_DESERIALIZER(pybind11::list);
MAKE_NLJSON_SERIALIZER_DESERIALIZER(pybind11::tuple);
MAKE_NLJSON_SERIALIZER_DESERIALIZER(pybind11::dict);

MAKE_NLJSON_SERIALIZER_ONLY(pybind11::handle);
MAKE_NLJSON_SERIALIZER_ONLY(pybind11::detail::item_accessor);
MAKE_NLJSON_SERIALIZER_ONLY(pybind11::detail::list_accessor);
MAKE_NLJSON_SERIALIZER_ONLY(pybind11::detail::tuple_accessor);
MAKE_NLJSON_SERIALIZER_ONLY(pybind11::detail::sequence_accessor);
MAKE_NLJSON_SERIALIZER_ONLY(pybind11::detail::str_attr_accessor);
MAKE_NLJSON_SERIALIZER_ONLY(pybind11::detail::obj_attr_accessor);

#undef MAKE_NLJSON_SERIALIZER
#undef MAKE_NLJSON_SERIALIZER_ONLY
} // namespace nlohmann

// pybind11 caster
namespace pybind11 {
namespace detail {
template <> struct type_caster<nlohmann::json> {
  public:
	PYBIND11_TYPE_CASTER(nlohmann::json, _("json"));

	bool load(handle src, bool) {
		try {
			value = pyjson::to_json(src);
			return true;
		} catch (...) {
			return false;
		}
	}

	static handle cast(nlohmann::json src, return_value_policy /* policy */,
	                   handle /* parent */) {
		object obj = pyjson::from_json(src);
		return obj.release();
	}
};

// --- NEW: caster for nlohmann::ordered_json ----------------------------
template <> struct type_caster<nlohmann::ordered_json> {
  public:
	PYBIND11_TYPE_CASTER(nlohmann::ordered_json, _("ordered_json"));

	bool load(handle src, bool) {
		try {
			// Python -> ordered_json (preserves insertion order)
			value = pyjson::to_ordered_json(src);
			return true;
		} catch (...) {
			return false;
		}
	}

	static handle cast(nlohmann::ordered_json src,
	                   return_value_policy /* policy */, handle /* parent */) {
		// ordered_json -> Python
		object obj = pyjson::from_json(src);
		return obj.release();
	}
};

} // namespace detail
} // namespace pybind11
