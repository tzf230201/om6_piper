# Operasi Piper: wake-up, state, maintenance, gerak

Panduan ini mengindeks implementasi yang ada di PC ini, bukan sertifikasi
keselamatan atau instruksi melewati gate yang gagal. Perubahan folder tidak
berarti robot siap bergerak. Jangan menjalankan dua pemilik CAN/RealSense bersamaan.
Perintah di bawah memakai `piper.cmd` dari folder `om6_piper`.

## Wake-up / mulai sesi

1. Operator memeriksa pemasangan mekanik, kabel, area gerak dan emergency stop
   fisik. Daya controller, koneksi USB dan enable motor adalah tiga keadaan
   berbeda. Melepas emergency stop bukan perintah home.
2. `piper.cmd doctor` hanya pemeriksaan offline. Pilih satu pemilik hardware:
   `piper.cmd gui --no-fgng --open` untuk monitor atau
   `piper.cmd state --seconds 5` untuk snapshot. Keduanya tidak mengirim
   enable/target; pembaca biasa dapat mengirim query identitas firmware.
   Tutup monitor sebelum sesi kontrol.
3. Periksa timestamp/freshness, semua driver, mode controller dan fault; jangan
   hanya melihat `arm_status.err_code == 0`. File lama bukan live feedback.
4. Preflight program commissioning yang sudah ada:

   ```powershell
   .\piper.cmd commission --mode preflight --output .\output\preflight_baru
   ```

   Folder harus baru. Preflight bukan enable atau bukti stop fisik. Mode
   `enable-stop`/`jog` tetap memerlukan flag operator dan gate lama. Launcher
   tidak menambahkan `--execute` atau mengubah `commissioning_interlock.json`.
5. Pada scan yang sebelumnya dijalankan operator, enable dilakukan di dalam
   `trajectory_test.py`, setelah konfirmasi dan pemeriksaan start/kamera.
   Tidak ada auto-enable ketika GUI/launcher dibuka.

Tidak ada skrip wake-up universal yang otomatis reset, clear, enable dan bergerak.
Program legacy `read_joint.py`, `gripper_test.py`, `gripper_piper_sdk.py` dan
`joint_raw_test.py` harus dibaca dahulu: beberapa mempunyai efek hardware pada
level module. Jangan mengimpornya untuk mengecek instalasi. Launcher tidak
mengimpor atau menjalankan program-program tersebut.
Skrip-skrip ini sekarang dipisahkan di `../examples/manual/`. Trajectory aktif
berada di `../src/trajectory_test.py`; adapter root tetap mendukung perintah lama.

## Clear error satu kali

`piper.cmd clear-error --help` memperlihatkan opsi program. Tanpa
`--execute-once`, program melakukan observasi/preflight dan query firmware:
bukan sepenuhnya offline, tetapi tidak mengirim clear. Operasi aktual tersedia:

```powershell
.\piper.cmd clear-error --execute-once --output .\output\clear_baru
```

Hanya setelah operator memeriksa penyebab fault dan menopang lengan. Policy lama
mensyaratkan idle/disabled/fresh dan mengirim maksimal satu
`clear_joint_error(255)`. Tidak ada enable, target, perubahan limit atau reset
otomatis. `flags_persist_no_retry` bukan berhasil; jangan mengulang diam-diam.
Simpan `before.json`, `result.json` dan observasi sesudahnya.

## Reset/recovery berbeda dari clear

`piper.cmd recover --help` mengarah ke `recover_piper_state_once.py`.
Tanpa `--execute-once` ia hanya preflight/observasi. Eksekusi memerlukan
`--operator-stable` dan pemeriksaan disabled/pose yang sudah ada. Program dapat
reset satu kali, menunggu Standby, lalu clear jika gate berikutnya lolos.
Reset bukan janji lengan tidak bergeser. Tidak ada reset otomatis setelah error.
Versi reset-only ada di `../piper_readonly/reset_piper_disabled_once.py`.
Permit dan catatan commissioning tidak diubah dalam konsolidasi ini.
File otoritatif tetap `../../piper_readonly/commissioning_interlock.json`.
Source yang pindah mengalami penyesuaian path, sehingga bukti izin lama yang
mengikat hash executor tidak dianggap valid untuk source baru. Tidak ada permit
yang diperbarui, diberikan, atau dipakai oleh migrasi.

## Disable dan stop

`piper.cmd disable` sendiri hanya menampilkan rencana secara offline.
Mode hardware memerlukan `--execute-once --operator-supported --output ...`.
Program memeriksa feedback sesudah TX; SDK return value saja tidak cukup.
Disable dapat menghilangkan torsi penahan dan lengan dapat turun. Damping stop,
disable dan pemutusan daya berbeda; menutup browser bukan menghentikan robot.
Ctrl+C pada scan bukan konfirmasi seluruh motor sudah disabled. Sistem ini
bukan emergency-stop software yang tersertifikasi.

## Scan meja, rekam visi dan map-live

Perintah berikut dapat menggerakkan robot setelah konfirmasi. Operator harus
meninjau plan dan area aktual, termasuk tinggi meja, gripper dan kabel:

```powershell
.\piper.cmd scan --table-plan ..\output\table_scan_20261007_v1\plan.json --map-live --vision-output .\output\table_scan_baru
```

Plan masih di workspace lama; output baru berada di `om6_piper`.
Viewer mapping: `http://127.0.0.1:8770/`. Recorder berbagi koneksi robot yang sama,
menunggu startup mapping sebelum enable dan tidak membuka CAN kedua. Preview CAD
bukan collision certification. Recorder tidak menimpa rekaman lama.

Driver firmware, backend CAN, satuan, limit, payload, policy reset/clear/disable,
interlock dan trajectory tidak diubah oleh launcher. Forwarding memakai argumen
operator dan console yang sama, tanpa auto-confirmation, background motion atau
retry. Tidak ada pembacaan maupun perintah hardware pada pekerjaan konsolidasi.
