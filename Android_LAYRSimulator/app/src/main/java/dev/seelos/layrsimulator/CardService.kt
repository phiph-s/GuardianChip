package dev.seelos.layrsimulator

import android.nfc.cardemulation.HostApduService
import android.os.Bundle

class CardService : HostApduService() {

    // AID bytes
    private val aid = hex("F0BAAAAAAD01")

    // Responses
    private val SW_OK = hex("9000")
    private val SW_INS_NOT_SUPPORTED = hex("6D00")
    private val SW_CLA_NOT_SUPPORTED = hex("6E00")

    // Static AUTH_INIT payload: 1010... pattern = 0xAA repeated
    private val authInitPayload = ByteArray(16) { 0xAA.toByte() }

    // (Optional) simple state for UI
    companion object {
        @Volatile var lastEvent: String = "Idle"
    }

    override fun processCommandApdu(apdu: ByteArray, extras: Bundle?): ByteArray {
        // 1) SELECT AID: 00 A4 04 00 Lc [AID]
        if (isSelectAid(apdu, aid)) {
            lastEvent = "SELECT ok"
            return SW_OK
        }

        if (apdu.size < 4) return SW_INS_NOT_SUPPORTED

        val cla = apdu[0]
        val ins = apdu[1]

        if (cla != 0x80.toByte()) return SW_CLA_NOT_SUPPORTED

        return when (ins) {
            0x10.toByte() -> { // AUTH_INIT
                lastEvent = "AUTH_INIT -> sent pattern"
                authInitPayload + SW_OK
            }
            else -> SW_INS_NOT_SUPPORTED
        }
    }

    override fun onDeactivated(reason: Int) {
        lastEvent = "Deactivated"
    }

    private fun isSelectAid(apdu: ByteArray, aid: ByteArray): Boolean {
        if (apdu.size < 5) return false
        if (apdu[0] != 0x00.toByte()) return false
        if (apdu[1] != 0xA4.toByte()) return false
        if (apdu[2] != 0x04.toByte()) return false
        if (apdu[3] != 0x00.toByte()) return false
        val lc = apdu[4].toInt() and 0xFF
        if (apdu.size < 5 + lc) return false
        val got = apdu.copyOfRange(5, 5 + lc)
        return got.contentEquals(aid)
    }

    private fun hex(s: String): ByteArray =
        s.chunked(2).map { it.toInt(16).toByte() }.toByteArray()
}
