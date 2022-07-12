package v

import chisel3._
import chisel3.util._

class LaneReq(param: VectorParameters) extends Bundle {
  val index: UInt = UInt(param.idBits.W)
  val uop: UInt = UInt(4.W)
  val w: Bool = Bool()
  val n: Bool =  Bool()
  val sew: UInt = UInt(2.W)
  val src: Vec[UInt] = Vec(3, UInt(param.ELEN.W))
  val groupIndex: UInt = UInt(param.groupSizeBits.W)
  val sign: Bool = Bool()
  val maskOP2: Bool = Bool()
  val maskDestination: Bool = Bool()
}

class LaneResp(param: VectorParameters) extends Bundle {
  val mask: UInt = UInt(4.W)
}

class LaneDecodeResult extends Bundle {
  // sub unit
  val logic: Bool = Bool()
  val arithmetic: Bool = Bool()
  val shift: Bool = Bool()
  val mul: Bool = Bool()
  val div: Bool = Bool()
  val poCount: Bool = Bool()
  val ffo: Bool = Bool()
  val getIndex: Bool = Bool()
  val dataProcessing: Bool = Bool()
  // operand
  // operand 0 is signed
  val s0: Bool = Bool()
  val s1: Bool = Bool()

  // - operand1?
  val sub1: Bool = Bool()

  val subUop: UInt = UInt(3.W)
}

class Lane(param: VectorParameters) extends Module {
  val req: DecoupledIO[LaneReq] = IO(Flipped(Decoupled(new LaneReq(param))))
  val resp: ValidIO[LaneResp] = IO(Valid(new LaneResp(param)))

  val decodeRes: LaneDecodeResult = WireInit(0.U.asTypeOf(new LaneDecodeResult))

  // sew
  /*val sewByteMask: UInt = (1.U << req.bits.sew).asUInt - 1.U
  val sewBitMask: UInt = FillInterleaved(8, sewByteMask)*/
  Seq.tabulate(4) {sew =>
    val significantBit = 1 << (sew + 3)
    val remainder = 64 - significantBit

    // handle sign bit
    val sign0 = req.bits.src(0)(significantBit - 1) && decodeRes.s0
    // operand sign correction
    val osc0 = Fill(remainder, sign0) ## req.bits.src(0)(significantBit - 1, 0)
    val sign1 = req.bits.src(1)(significantBit - 1) && decodeRes.s1
    val osc1 = Fill(remainder, sign1) ## req.bits.src(1)(significantBit - 1, 0)
    /*val sign2 = req.bits.src(2)(significantBit - 1)
    val osc2 = Fill(remainder, sign2) ## req.bits.src(2)(significantBit - 1, 0)*/

    // op0 - op1
    val SubtractionOperand: UInt = osc1 ^ Fill(64, decodeRes.sub1)


  }
  resp <> DontCare
  req <> DontCare
}
