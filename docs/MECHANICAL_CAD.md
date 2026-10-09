# Mekanik, CAD dan koordinat kamera

## Koleksi CAD

`../cad/source/arm_with_wirst_mount_d405.stp` adalah salinan identik STEP pengguna.
`../cad/inventor/` berisi IAM, empat IPT yang tercatat di laporan Inventor lama,
dan STL arm pendukung. `../cad/reference/` menyimpan salinan referensi D405 yang
sudah dipakai. Hash dan lokasi asal ada di `../cad/copy_manifest.json`.
Asli di proyek `kubota_lab/moonshot_demo` tidak dipindahkan atau diedit.

Ini koleksi direct dependencies berdasarkan `inventor_assembly.json`, **bukan
Inventor Pack and Go yang telah diverifikasi**. Referensi tersimpan di IAM dapat
tetap mengarah ke path asli; lakukan resolve ke IPT lokal saat membuka salinan.
Tidak ada Inventor COM, regenerate, perubahan constraint atau save CAD pada tugas ini.

STEP hanya berisi tiga solid kamera/mounting. Komponen Piper dalam STEP tidak
menyimpan solid robot. Mesh arm berasal dari STL/URDF terpisah; jangan menyebut
STEP ini assembly robot lengkap.

## Model aktif di viewer

Lokasi yang masih dipakai: `../../piper_readonly/piper_model_source/` untuk URDF
dan mesh robot, `../piper_readonly/assets/wrist_cad/` di dalam `om6_piper` untuk
tiga bagian D405. Jalur aset wrist lama mengarah ke folder baru yang sama.
`attachment.json` menentukan pose terhadap `gripper_base`; `fitted_mount.json`
menentukan koreksi pemasangan viewer yang aktif. Mount dan kamera ikut joint 6,
bukan bukaan jari. `bottom_mount_flip.json` menyimpan eksperimen reversal lama.

Koreksi mesh bawah mengubah pitch lubang 12,5 menjadi 12 mm dan recess 0,6 mm
agar cocok dengan tab pada mesh resmi. Kamera/mount atas bergeser sebagai satu
benda kaku agar engsel sejajar. Ini **mesh turunan untuk visualisasi**, bukan
dimensi manufaktur yang disetujui, sertifikasi collision atau kalibrasi hand-eye.
CAD asli, STL hasil konversi awal dan laporan fit tetap dipertahankan.

Penjelasan lengkap: [provenance CAD](../piper_readonly/assets/wrist_cad/README.md).
Tool: `import_wrist_step.mjs`, `inspect_inventor.ps1`, `register_wrist_cad.py`,
`refine_wrist_registration.py`, `fit_wrist_mount.py`, `test_fitted_mount.py`.
Jangan menjalankan generator untuk sekadar memeriksa path karena beberapa
menulis ulang aset/konfigurasi. Tes yang benar-benar offline dipisahkan.

## Transform kamera

Model dasar:

`T_base_optical(q) = T_base_gripper(q) × T_gripper_camera_body × T_body_optical`.

Estimasi nominal D405 ada di
`../../output/cad_optical_20261005_v1/fgng_config.cad_nominal.json`:
`body_from_depth_optical.xyz = [0.009, 0, -0.0038] m`,
`rpy = [0, 0, pi] rad`. Optical X/Y berlawanan CAD X/Y; Z tetap searah.
Itu perkiraan CAD + mekanik vendor, dengan `calibrated: false`, bukan hasil
pengukuran hand-eye atau error bound.

Viewer lama mengangkat visual robot +20 mm pada world Z; mode F-GNG lama juga
memiliki world offset yang dikonfigurasi. Replay/scan meja yang lebih baru memakai
base frame fisik: `table_scan.configuration()` mengeset offset mapping menjadi
`[0,0,0]`, terpisah dari lift tampilan. Jangan menambah 20 mm kedua kali, mengubah
CAD, atau menganggap nilai display sebagai posisi fisik.

Plan meja ada di `../../output/table_scan_20261007_v1/plan.json`. Tinggi meja
adalah asumsi plan; mesh preview bukan pemeriksaan lengkap kabel, jari, lingkungan
atau self-collision. Kinematika, kalibrasi dan mounting tidak diubah oleh konsolidasi.
