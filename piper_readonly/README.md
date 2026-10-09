# Aplikasi Piper dan GUI

Ini lokasi source aplikasi aktif setelah migrasi 9 Oktober 2026.
Mulai dari [panduan utama](../docs/START_HERE.md). Jalankan launcher
`../piper.cmd`; gunakan `doctor` untuk cek tanpa hardware.

`web/` berisi GUI dan pengujian frontend; `assets/wrist_cad/` berisi mesh D405.
`app_paths.py` menyatakan dependency bersama secara eksplisit. State/interlock,
SDK, environment, URDF resmi dan arsip riset tidak digandakan di sini.

Perintah developer: `npm test` dan `npm run build`. Keduanya tidak menggerakkan
robot. Dependency Node tetap menggunakan folder bersama melalui junction.
Tes lengkap serta laporan migrasi ada di [status migrasi](../docs/MIGRATION.md).

Jangan menjalankan backup source di `../.local/migration_app_20261009/`; backup hanya
untuk audit/pemulihan. Source aktif diedit di folder ini, bukan pengarah legacy.
