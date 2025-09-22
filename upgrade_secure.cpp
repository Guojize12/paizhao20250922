#include "upgrade_secure.h"
#include <Preferences.h>
#include <mbedtls/sha256.h>
#include <mbedtls/pk.h>

static OtaSecureCtx g;
static Preferences pref;
static const char* NS = "upgsec";

// 简单 32bit FNV-1a 哈希（给 version_tag 用）
static uint32_t fnv1a(const char* s){
  uint32_t h = 2166136261u;
  while(*s){ h ^= (uint8_t)*s++; h *= 16777619u; }
  return h;
}

static void sha_init(){
  auto* ctx = new mbedtls_sha256_context();
  mbedtls_sha256_init(ctx);
  mbedtls_sha256_starts(ctx, 0); // 0=SHA-256
  g.sha_ctx = ctx;
}
static void sha_update(const uint8_t* p, size_t n){
  mbedtls_sha256_update((mbedtls_sha256_context*)g.sha_ctx, p, n);
}
static void sha_finish(){
  mbedtls_sha256_finish((mbedtls_sha256_context*)g.sha_ctx, g.sha_calc);
  mbedtls_sha256_free((mbedtls_sha256_context*)g.sha_ctx);
  delete (mbedtls_sha256_context*)g.sha_ctx;
  g.sha_ctx = nullptr;
}

static bool parse_footer_from_tail(OtaFooterView& f){
  // 从尾缓冲中向前搜索 "OTAF"
  if(g.tail_len < 9+32) return false;
  const uint8_t* base = g.tail;
  for(int off = (int)g.tail_len - 9 - 32; off >= 0; --off){
    if(base[off]=='O' && base[off+1]=='T' && base[off+2]=='A' && base[off+3]=='F'){
      f.base = base + off;
      f.ver = f.base[4];
      f.algo = f.base[5];
      f.digest_len = f.base[6];
      f.sig_len = (uint16_t)f.base[7] << 8 | f.base[8];
      if(f.ver != 1 || f.algo != 1 || f.digest_len != 32) return false;
      if(off + 9 + 32 + f.sig_len > (int)g.tail_len) return false;
      f.digest = f.base + 9;
      f.sig = f.base + 9 + 32;
      return true;
    }
  }
  return false;
}

bool upgsec_begin(uint32_t total_size, const char* version_tag, uint32_t block_size){
  upgsec_reset();
  g.total_size = total_size;
  g.block_size = block_size ? block_size : OTA_BITMAP_BLOCK_SIZE;
  g.blocks_total = (total_size + g.block_size - 1) / g.block_size;
  g.bitmap_bytes = (g.blocks_total + 7) / 8;
  g.bitmap = (uint8_t*)heap_caps_malloc(g.bitmap_bytes, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
  if(!g.bitmap) g.bitmap = (uint8_t*)malloc(g.bitmap_bytes);
  if(!g.bitmap) return false;
  memset(g.bitmap, 0, g.bitmap_bytes);
  g.tag_hash = fnv1a(version_tag ? version_tag : "no-tag");
  sha_init();
  return true;
}

bool upgsec_load_bitmap(uint32_t total_size, const char* version_tag, uint32_t block_size){
  if(!upgsec_begin(total_size, version_tag, block_size)) return false;
  // 加载持久化位图（可选）
  if(!pref.begin(NS, true)) return true; // 打不开就当没有
  char key[16];
  snprintf(key, sizeof(key), "bm_%08lx", (unsigned long)g.tag_hash);
  size_t n = pref.getBytesLength(key);
  if(n == g.bitmap_bytes){
    pref.getBytes(key, g.bitmap, g.bitmap_bytes);
  }
  pref.end();
  return true;
}

bool upgsec_save_bitmap(){
  if(!pref.begin(NS, false)) return false;
  char key[16];
  snprintf(key, sizeof(key), "bm_%08lx", (unsigned long)g.tag_hash);
  pref.putBytes(key, g.bitmap, g.bitmap_bytes);
  pref.end();
  return true;
}

bool upgsec_mark_blocks(uint32_t file_offset, uint32_t len){
  if(g.block_size == 0) return false;
  uint32_t first = file_offset / g.block_size;
  uint32_t last  = (file_offset + len - 1) / g.block_size;
  if(last >= g.blocks_total) last = g.blocks_total - 1;
  for(uint32_t i=first; i<=last; ++i){
    g.bitmap[i >> 3] |= (uint8_t)(1u << (7 - (i & 7)));
  }
  return true;
}

bool upgsec_is_complete(){
  // 所有块都置位
  for(uint32_t i=0;i<g.blocks_total;i++){
    if(((g.bitmap[i >> 3] >> (7 - (i & 7))) & 1u) == 0) return false;
  }
  return true;
}

static bool write_and_hash(const uint8_t* p, uint32_t n,
                           size_t (*ota_write_cb)(const uint8_t*, size_t),
                           String& err){
  if(n == 0) return true;
  size_t w = ota_write_cb(p, n);
  if(w != n){ err = "ota_write short"; return false; }
  sha_update(p, n);
  g.content_written += n;
  return true;
}

bool upgsec_update(uint32_t file_offset, const uint8_t* data, uint32_t len, bool is_last,
                   size_t (*ota_write_cb)(const uint8_t*, size_t), String& err){
  // 断言：外部以顺序偏移提交（推荐）。若乱序，请先在外部重组后再调用。
  (void)file_offset;
  g.received += len;

  // 如果尾缓冲 + 新数据会超限，则把尾缓冲前部写出去（保留最后 OTA_FOOTER_MAX_SIZE 字节）
  uint32_t capacity = OTA_FOOTER_MAX_SIZE;
  if(g.tail_len + len > capacity){
    uint32_t to_write = (g.tail_len + len) - capacity;
    if(to_write > g.tail_len) to_write = g.tail_len; // 仅能写已有的
    if(to_write){
      if(!write_and_hash(g.tail, to_write, ota_write_cb, err)) return false;
      memmove(g.tail, g.tail + to_write, g.tail_len - to_write);
      g.tail_len -= to_write;
    }
  }
  // 追加新数据到尾缓冲
  if(len){
    memcpy(g.tail + g.tail_len, data, len);
    g.tail_len += len;
  }

  // 标记位图
  upgsec_mark_blocks(file_offset, len);

  if(is_last){
    // 解析 Footer
    if(!parse_footer_from_tail(g.footer)){
      err = "footer parse fail";
      return false;
    }
    // 将尾缓冲的“内容部分”写入 OTA（不包含 Footer）
    uint32_t content_tail_len = g.tail_len - g.footer.total_len();
    if(!write_and_hash(g.tail, content_tail_len, ota_write_cb, err)) return false;

    // 完成 SHA
    sha_finish();

    // 比对 digest
    if(memcmp(g.sha_calc, g.footer.digest, 32) != 0){
      err = "sha256 mismatch";
      return false;
    }
  }
  return true;
}

bool upgsec_finalize_verify(const char* pubkey_pem, String& err){
  // 验签（ECDSA P-256，DER 签名）
  mbedtls_pk_context pk;
  mbedtls_pk_init(&pk);
  int rc = mbedtls_pk_parse_public_key(&pk, (const unsigned char*)pubkey_pem, strlen(pubkey_pem) + 1);
  if(rc != 0){ mbedtls_pk_free(&pk); err = "pubkey parse fail"; return false; }

  rc = mbedtls_pk_verify(&pk, MBEDTLS_MD_SHA256, g.sha_calc, 0, g.footer.sig, g.footer.sig_len);
  mbedtls_pk_free(&pk);
  if(rc != 0){ err = "signature verify fail"; return false; }

  return true;
}

void upgsec_get_progress(uint32_t& received_total, uint32_t& content_written){
  received_total = g.received;
  content_written = g.content_written;
}

void upgsec_reset(){
  if(g.sha_ctx){
    mbedtls_sha256_free((mbedtls_sha256_context*)g.sha_ctx);
    delete (mbedtls_sha256_context*)g.sha_ctx;
    g.sha_ctx = nullptr;
  }
  if(g.bitmap){ free(g.bitmap); g.bitmap=nullptr; }
  memset(&g, 0, sizeof(g));
}