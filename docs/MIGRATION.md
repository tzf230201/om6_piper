# Migrasi aplikasi Piper — 9 Oktober 2026

**Pembaruan layout repository:** source mapping/trajectory sekarang berada di
`src/`, tes di `tests/`, dan audit migrasi dipindahkan ke `.local/`. Nama folder
audit dalam catatan historis di bawah adalah nama awal; lihat
[struktur terkini](REPOSITORY.md) dan panduan utama untuk perintah aktif.

## Hasil tahap aplikasi/GUI

Tahap sempit ini telah selesai, sesuai persetujuan pengguna untuk memindahkan
aplikasi dahulu dan membiarkan arsip riset bersama di lokasi lama. Tidak ada
pembacaan CAN/USB/kamera, enable, reset, clear error atau gerakan fisik.

| Komponen | Lokasi relatif terhadap `om6_piper` |
| --- | --- |
| 24 modul aplikasi, state reader, maintenance, gateway | `piper_readonly/` |
| GUI, Monitor J1–J6, preview trajectory dan bundle | `piper_readonly/web/` |
| Mesh serta konfigurasi mounting D405 aktif | `piper_readonly/assets/wrist_cad/` |
| Trajectory, capture dan map-live yang sudah ada | root `om6_piper/` |
| Panduan mekanik, CAD, operasi dan state | `docs/` dan `cad/` |
| Satu interlock dan permit/claim historis | tetap `../piper_readonly/commissioning_interlock.json` dan path evidence asal |
| SDK, venv, URDF/mesh resmi, mapping bersama | tetap `../piper_readonly/` |
| C++/F-GNG, dataset, paper, output riset dan video bersama | tetap di workspace asal |

Daftar modul lengkap: `../migration_app_20261009/plan.json`.
`app_paths.py` menyatakan batas aplikasi/dependency. `robot_model.ROOT` tetap
menunjuk data bersama untuk kompatibilitas mapping; web dan wrist assets
menggunakan `APP_ROOT` yang baru secara eksplisit.

### Kompatibilitas dan backup

- File `.py` lama kini pengarah kecil ke implementasi baru, bukan kontrol kedua.
  Namespace modul dipertahankan agar import, CLI dan mocking tetap bekerja.
- `../piper_readonly/web` dan `../piper_readonly/assets` adalah junction ke
  folder baru. Aset asli disimpan di backup, tidak dihapus.
- `piper_readonly/node_modules` adalah junction ke dependency Node lama.
  Tidak ada venv, SDK, repository vendor atau dataset yang dipindahkan.
- Interlock dan `fgng_config.json` identik dengan sebelum migrasi (SHA-256).
  Tidak ada gate kedua, permit baru, konsumsi claim, atau reset izin. Penyesuaian
  path mengubah hash beberapa executor; izin berbasis hash lama tidak dibawa maju.
- `piper.cmd` sekarang memakai `.venv` yang memiliki `usb`/`gs_usb`, bukan
  `.venv_mapping` yang tidak memiliki dua modul itu. Tidak ada instalasi baru.

Backup ada di `../migration_app_20261009/original_sources/` dan
`original_directories/`. Itu arsip, **bukan entry point untuk dijalankan**.
Pemulihan perlu meninjau manifest/hash/junction, bukan menjalankan migrasi ulang
atau menghapus junction secara rekursif. Folder `../piper_readonly` tetap
dibutuhkan; proyek belum portable sebagai satu folder.

### Verifikasi tahap ini

Sebelum cutover: **192 tes Python + 26 tes JavaScript lulus**. Verifikasi akhir
sesudah cutover diperluas menjadi **216 tes Python + 26 tes JavaScript**, semuanya
lulus, termasuk 24 tes integrasi recorder, scan meja, print joint dan mapping C++.
Termasuk freshness, state, policy reset/clear/disable, izin one-shot, HTTP
read-only, gateway, launcher, kinematika, CAD dan preview trajectory. Model untuk
dua versi firmware dan versi tidak diketahui, transform optik, serta semua mesh
identik sebelum/sesudah. Bundle GUI tidak diubah. Build esbuild dari source baru
berhasil ke file audit terpisah, tanpa menimpa bundle aktif. `doctor` dan CLI help
melalui jalur baru maupun jalur lama berhasil.

USB discovery dan pipeline RealSense diblokir selama suite offline. Tes HTTP
memakai server snapshot sementara di loopback. Delapan tes socket semula ditolak
sandbox (WinError 10013), lalu lulus dengan izin loopback. Migrasi sempit dan
junction diberi izin terpisah. Tes ini bukan bukti kesiapan hardware.

Bukti: `offline_staging_verified.json/.log`, `offline_post_verified.json/.log`,
`*_javascript.log`, `model_before.json`, `source_hashes.json`,
`protected_hashes.json`, `application_path_changes.diff`, dan `completed.json`
di `../migration_app_20261009/`. Tidak ada restart viewer live atau perubahan
algoritma mapping. Tahap shared dependency berikutnya perlu cakupan tersendiri.

## Arsip tahap sebelumnya — bukan status terkini

Catatan di bawah dipertahankan untuk menjelaskan percobaan luas yang gagal.
Rencana luas itu tetap dinonaktifkan. Migrasi aplikasi di atas memakai skrip
baru `migrate_piper_app.ps1`, bukan mengulang rencana yang ditolak.

Permintaan: satukan program/GUI/mekanik/CAD/operasi dan state Piper di `om6_piper`.

## Sudah dilakukan

- Inventaris komponen dan dependensi; backup source sebelum perubahan ada di
  `../migration_20261009/original_sources`, hash di `protected_hashes.json`.
- Launcher `piper.cmd`, `piper.py` dan `run_viewer.cmd`; default doctor offline.
- Panduan start, wake-up/maintenance, seluruh kelompok state yang dibaca, dan CAD.
- Salinan 15 file CAD/referensi beserta SHA-256; file asli tidak diubah.

## Belum dipindahkan

`piper_readonly`, C++/F-GNG/research, environment, dataset dan output workspace
masih di tempat semula. Percobaan pertama gagal pada pemindahan direktori
`piper_readonly` karena Windows melaporkan sedang dipakai. Tidak ada direktori
tujuan atau junction yang terbentuk. Permintaan menjalankan migrasi massal dengan
akses di luar sandbox ditolak auto-review karena terlalu luas dan berpotensi
mengganggu arsip/dependensi bersama. Tidak dicoba melalui jalur lain.

`consolidate_piper.ps1` dan `plan.json` adalah rencana yang **belum diterapkan**;
eksekusi skrip luas itu sekarang dinonaktifkan secara eksplisit,
bukan sesuatu yang harus dijalankan untuk memakai launcher. Tidak ada rollback
data yang diperlukan. Jangan resume rencana luas itu tanpa peninjauan cakupan
dan izin baru. Blueprint lama tidak berarti seluruh direktori tersebut harus
menjadi milik Piper.

## Langkah yang masih perlu persetujuan

1. Migrasi khusus aplikasi Piper/GUI dahulu, setelah pemilik file yang mengunci
   direktori diketahui dan ditutup secara normal oleh operator.
2. Pertahankan arsip paper/dataset bersama di luar tahap pertama. Inventaris
   referensi executable/environment dan file kontrol sebelum mengganti path.
3. Staging dengan hash dan tes offline, lalu cutover satu kelompok file; jangan
   menggandakan interlock/permit menjadi dua otoritas kontrol aktif.
4. Setelah aplikasi lolos, tentukan folder mapping/rekaman mana yang memang
   dipindahkan dan mana tetap shared dependency. Baru lakukan cleanup dengan
   persetujuan, tanpa menghapus lewat junction rekursif.

Launcher transisi menyelesaikan resource lokal bila tersedia, lalu lokasi lama.
GUI dan maintenance sekarang masih memakai program asli dan state/interlock asli.
Launcher tidak melewati gate, tidak mengirim perintah otomatis, dan tidak memberi
klaim bahwa robot sedang hidup/sehat. Tidak ada motor, kamera, reset atau clear
error yang diakses saat menyusun panduan dan memeriksa konsolidasi ini.

## Pemeriksaan akhir

Enam tes launcher dan sembilan tes GUI offline lulus; `doctor` menemukan semua
dependency yang didaftarkan. Ketiga tes HTTP awalnya dibatasi socket sandbox
(WinError 10013), lalu lulus dengan izin khusus untuk verifikasi loopback saja.
Izin tes tersebut bukan izin untuk migrasi massal. Hash source kontrol, interlock,
aset GUI dan native DLL tetap cocok dengan baseline; satu-satunya file lama yang
disunting adalah pengantar `om6_piper/README.md`. Semua 15 salinan CAD/reference
cocok dengan hash sumber. Hasil lengkap: `../migration_20261009/verification.json`.
