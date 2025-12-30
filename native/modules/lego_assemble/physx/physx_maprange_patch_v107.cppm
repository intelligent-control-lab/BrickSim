module;

#include <cstddef>
#include <link.h>
#include <sys/mman.h>
#include <unistd.h>

export module lego_assemble.physx.physx_maprange_patch_v107;

import std;
import lego_assemble.utils.logging;
import lego_assemble.vendor;

namespace {

using namespace lego_assemble;

using UsdPrimMap = std::map<const pxr::SdfPath, pxr::UsdPrim>;

// Memory layout for v107 PrimIteratorMapRange (derived from disassembly):
//   0x00 vptr
//   0x08 bool mAtEnd
//   0x10 const UsdPrimMap& (stored as pointer)
//   0x18 UsdPrimMap::const_iterator
//   0x20 pxr::UsdPrimRange mRange
//   0x58 pxr::UsdPrimRange::const_iterator mIter
struct PrimIteratorMapRangeLayout {
	void *vptr;
	bool mAtEnd;
	std::byte _pad0[7];

	const UsdPrimMap *mPrimMap;
	UsdPrimMap::const_iterator mPrimMapIter;

	pxr::UsdPrimRange mRange;
	pxr::UsdPrimRange::const_iterator mIter;
};

#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Winvalid-offsetof"
#endif

static_assert(offsetof(PrimIteratorMapRangeLayout, mAtEnd) == 0x08);
static_assert(offsetof(PrimIteratorMapRangeLayout, mPrimMap) == 0x10);
static_assert(offsetof(PrimIteratorMapRangeLayout, mPrimMapIter) == 0x18);
static_assert(offsetof(PrimIteratorMapRangeLayout, mRange) == 0x20);
static_assert(offsetof(PrimIteratorMapRangeLayout, mIter) == 0x58);

#if defined(__clang__)
#pragma clang diagnostic pop
#endif

void _advanceToNextNonEmptyRange(PrimIteratorMapRangeLayout *self) {
	self->mAtEnd = true;

	while (self->mPrimMapIter != self->mPrimMap->end()) {
		self->mRange = pxr::UsdPrimRange(self->mPrimMapIter->second,
		                                 pxr::UsdTraverseInstanceProxies());
		self->mIter = self->mRange.begin();

		if (self->mIter != self->mRange.end()) {
			self->mAtEnd = false;
			return;
		}

		++self->mPrimMapIter;
	}

	self->mAtEnd = true;
	self->mIter = self->mRange.end();
}

// Must match vtable call ABI: function pointer taking `this` in RDI.
extern "C" void patched_reset(void *obj) {
	auto *self = reinterpret_cast<PrimIteratorMapRangeLayout *>(obj);
	self->mPrimMapIter = self->mPrimMap->begin();
	_advanceToNextNonEmptyRange(self);
}

extern "C" void patched_next(void *obj) {
	auto *self = reinterpret_cast<PrimIteratorMapRangeLayout *>(obj);

	if (self->mAtEnd)
		return;

	++self->mIter;
	if (self->mIter != self->mRange.end())
		return;

	++self->mPrimMapIter;
	_advanceToNextNonEmptyRange(self);
}

struct ModuleInfo {
	std::uintptr_t base = 0;
	const char *name = nullptr;
};

int find_v107_physx(struct dl_phdr_info *info, size_t, void *data) {
	auto *out = reinterpret_cast<ModuleInfo *>(data);
	if (!info->dlpi_name || !*info->dlpi_name)
		return 0;

	std::string_view n{info->dlpi_name};
	if (n.find("libomni.physx.plugin.so") == std::string_view::npos)
		return 0;

	if (n.find("omni.physx-107.3.26+107.3.3") == std::string_view::npos)
		return 0;

	out->base = static_cast<std::uintptr_t>(info->dlpi_addr);
	out->name = info->dlpi_name;
	return 1;
}

bool make_writable(void *addr, size_t len) {
	const long pageSize = sysconf(_SC_PAGESIZE);
	const std::uintptr_t p = reinterpret_cast<std::uintptr_t>(addr);
	const std::uintptr_t start =
	    p & ~(static_cast<std::uintptr_t>(pageSize) - 1);
	const std::uintptr_t end =
	    (p + len + static_cast<std::uintptr_t>(pageSize) - 1) &
	    ~(static_cast<std::uintptr_t>(pageSize) - 1);

	return ::mprotect(reinterpret_cast<void *>(start), end - start,
	                  PROT_READ | PROT_WRITE) == 0;
}

} // namespace

__attribute__((constructor)) void install_patch() {
	ModuleInfo mod;
	dl_iterate_phdr(&find_v107_physx, &mod);
	if (!mod.base) {
		log_error("physx_maprange_patch_v107: omni.physx not loaded yet, or "
		          "different version");
		return;
	}

	constexpr std::uintptr_t kVtableOffset = 0x0a90bd8;
	constexpr std::uintptr_t kResetOffset = 0x002c7ba0;
	constexpr std::uintptr_t kNextOffset = 0x002c78c0;

	auto *vtable = reinterpret_cast<void **>(mod.base + kVtableOffset);

	// Safety: only patch if the vtable looks exactly like v107.
	if (vtable[4] != reinterpret_cast<void *>(mod.base + kResetOffset)) {
		log_error("physx_maprange_patch_v107: vtable reset function mismatch, "
		          "not patching");
		return;
	}
	if (vtable[7] != reinterpret_cast<void *>(mod.base + kNextOffset)) {
		log_error("physx_maprange_patch_v107: vtable next function mismatch, "
		          "not patching");
		return;
	}

	if (!make_writable(vtable, 8 * 9)) {
		log_error("physx_maprange_patch_v107: failed to make vtable writable");
		return;
	}

	vtable[4] = reinterpret_cast<void *>(&patched_reset);
	vtable[7] = reinterpret_cast<void *>(&patched_next);
	log_info("physx_maprange_patch_v107: patch installed successfully");
}
