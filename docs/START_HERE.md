# Piper: mulai dari sini

**Tahap aplikasi/GUI selesai.** Launcher, 24 modul aplikasi Piper, GUI, aset
wrist, panduan dan salinan CAD ada di `ICRA2027/om6_piper`. Jalur aplikasi lama
tetap bekerja melalui pengarah kompatibilitas. SDK, environment, state/interlock,
model robot resmi dan arsip riset tetap di lokasi bersama. Folder ini belum
portable jika disalin sendirian ke PC lain.

```text
om6_piper/
  piper.cmd / piper.py       satu pintu menjalankan program yang sudah ada
  run_viewer.cmd            GUI mesh 3D + menu Monitor
  src/                     trajectory, capture, mapping, replay dan eksperimen
  tests/                   tes offline, terpisah dari contoh hardware
  examples/manual/         skrip hardware historis; bukan unit test
  trajectory_test.py       adapter kompatibilitas ke src/trajectory_test.py
  piper_readonly/           source aplikasi, pembaca state, maintenance, gateway
    web/                   viewer 3D, tabel Monitor, preview trajectory
    assets/wrist_cad/       mesh dan konfigurasi mounting D405 aktif
    app_paths.py           batas antara aplikasi lokal dan resource bersama
  docs/                     operasi, state, mekanik, dan migrasi
  cad/source/               STEP asli
  cad/inventor/             salinan IAM dan empat IPT rujukannya + STL arm
  cad/reference/            referensi D405 dan model optik vendor
  output/                   rekaman robot yang memang sudah ada di sini
  cad/copy_manifest.json     lokasi asal dan hash salinan CAD
  .local/                  backup, audit migrasi dan laporan tes (diabaikan Git)
```

## Cek instalasi tanpa mengakses robot

```powershell
cd "C:\Users\teuku\OneDrive\Documents\ChatGPT\ICRA2027\om6_piper"
.\piper.cmd doctor
.\piper.cmd --help
```

`doctor` memeriksa file, modul dan mencetak lokasi aktual. Ia tidak menyalakan motor,
menghapus error, menghubungkan CAN, atau membuka kamera.

## Viewer

Jalankan `run_viewer.cmd`, atau `piper.cmd gui --open`. Launcher meneruskan ke
`piper_readonly/web_monitor.py` di dalam `om6_piper`, tanpa membuat salinan state kontrol.
Ini membuka pembaca robot
dan kamera sesuai perilaku viewer yang sudah ada. Untuk robot tanpa kamera,
gunakan `piper.cmd gui --no-fgng --open`. URL lokal adalah
`http://127.0.0.1:8765/`; menu **Monitor** berisi tabel J1--J6.
Jangan menjalankan pembaca CAN/kamera kedua bersamaan dengan trajectory.
Konfigurasi Tailscale lama dipertahankan, bukan dikonfigurasi ulang oleh migrasi.

Untuk inspeksi offline (tidak membuka CAN atau kamera):

```powershell
.\piper.cmd gui --snapshot ..\output\motion_preflight_20261005_v1\robot_status.json --port 8768 --open
```

Tampilan ini berlabel snapshot, **bukan kondisi robot saat ini**. Snapshot lama
dapat berisi fault yang sudah tidak berlaku; jangan menyimpulkan readiness dari situ.

## Panduan lanjutan

- [Wake-up, clear error, disable dan gerak](ROBOT_OPERATIONS.md)
- [Arti seluruh kelompok state yang kita baca](ROBOT_STATE.md)
- [Mekanik, CAD dan transform kamera](MECHANICAL_CAD.md)
- [Apa yang dipindahkan dan kompatibilitas path](MIGRATION.md)
- [Catatan eksperimen dan mapping](research/README.md)
- [Struktur repository dan dependency](REPOSITORY.md)
- [Dokumentasi monitor terdahulu](../../piper_readonly/README.md)

Masih eksternal: `../piper_readonly` (SDK, environment, URDF/mesh resmi, state,
mapping bersama, alat CAD historis dan log), `../cpp_surface`, `../fgng_realsense`,
`../fgng_upstream`, `../research_20260930`, `../output`, dan `../tmp/video_tools`.
Jalur `../piper_readonly/web` dan `assets` adalah junction ke aset aplikasi baru,
bukan salinan aktif kedua. Jangan hapus direktori lama: masih menyimpan dependency.
Argumen path relatif launcher dihitung dari `om6_piper`, bukan lokasi terminal.
Interlock dan permit one-shot tetap di lokasi asal, tidak direset.

`piper.cmd` menggunakan `../piper_readonly/.venv`, karena environment tersebut
memiliki `usb` dan `gs_usb` yang dibutuhkan aplikasi robot. `.venv_mapping`
tetap tersedia untuk workflow riset lama tetapi tidak memiliki kedua modul itu.
Tidak ada paket yang diinstal ulang dalam migrasi ini.

Untuk developer GUI: `cd piper_readonly`, lalu `npm test` atau `npm run build`.
Dependency JavaScript tetap dibagi dari `../../piper_readonly/node_modules`.
Pengujian lengkap migrasi, dari folder `om6_piper`:

```powershell
.\piper.cmd test
```

Suite tersebut tidak membuka hardware.
