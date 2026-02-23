package com.wallter.app.ble

import java.nio.ByteBuffer
import java.nio.ByteOrder

data class WallterStatus(
    val state: Int,
    val lastErr: Int,
    val bytesWritten: Long,
    val imageSize: Long,
    val crc32: Long,
    val sha256: ByteArray,
) {
    companion object {
        const val SIZE_BYTES: Int = 48

        fun parse(payload: ByteArray): WallterStatus? {
            if (payload.size < SIZE_BYTES) return null
            val bb = ByteBuffer.wrap(payload).order(ByteOrder.LITTLE_ENDIAN)
            val state = bb.get(0).toInt() and 0xFF
            val lastErr = bb.get(1).toInt() and 0xFF
            val bytesWritten = bb.getInt(4).toLong() and 0xFFFF_FFFFL
            val imageSize = bb.getInt(8).toLong() and 0xFFFF_FFFFL
            val crc32 = bb.getInt(12).toLong() and 0xFFFF_FFFFL
            val sha = ByteArray(32)
            bb.position(16)
            bb.get(sha)
            return WallterStatus(
                state = state,
                lastErr = lastErr,
                bytesWritten = bytesWritten,
                imageSize = imageSize,
                crc32 = crc32,
                sha256 = sha,
            )
        }
    }
}
