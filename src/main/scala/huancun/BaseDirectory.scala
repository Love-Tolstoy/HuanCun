/** *************************************************************************************
  * Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
  * Copyright (c) 2020-2021 Peng Cheng Laboratory
  *
  * XiangShan is licensed under Mulan PSL v2.
  * You can use this software according to the terms and conditions of the Mulan PSL v2.
  * You may obtain a copy of Mulan PSL v2 at:
  *          http://license.coscl.org.cn/MulanPSL2
  *
  * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
  * EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
  * MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
  *
  * See the Mulan PSL v2 for more details.
  * *************************************************************************************
  */

// See LICENSE.SiFive for license details.

package huancun

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import chisel3.util.random.LFSR
import freechips.rocketchip.tilelink.TLMessages
import freechips.rocketchip.util.{Pow2ClockDivider, ReplacementPolicy}
import huancun.utils._
import utility.{Code}

trait BaseDirResult extends HuanCunBundle {
  val idOH = UInt(mshrsAll.W) // which mshr the result should be sent to
}
trait BaseDirWrite extends HuanCunBundle
trait BaseTagWrite extends HuanCunBundle

class DirRead(implicit p: Parameters) extends HuanCunBundle {
  val idOH = UInt(mshrsAll.W)
  val tag = UInt(tagBits.W)
  val set = UInt(setBits.W)
  val replacerInfo = new ReplacerInfo()
  val source = UInt(sourceIdBits.W)
  val wayMode = Bool()
  val way = UInt(log2Ceil(maxWays).W)
}

abstract class BaseDirectoryIO[T_RESULT <: BaseDirResult, T_DIR_W <: BaseDirWrite, T_TAG_W <: BaseTagWrite](
  implicit p: Parameters)
    extends HuanCunBundle {
  val read:    DecoupledIO[DirRead]
  val result:  Valid[T_RESULT]
  val dirWReq: DecoupledIO[T_DIR_W]
  val tagWReq:  DecoupledIO[T_TAG_W]
}

abstract class BaseDirectory[T_RESULT <: BaseDirResult, T_DIR_W <: BaseDirWrite, T_TAG_W <: BaseTagWrite](
  implicit p: Parameters)
    extends HuanCunModule {
  val io: BaseDirectoryIO[T_RESULT, T_DIR_W, T_TAG_W]
}

class SubDirectory[T <: Data](
  wports:      Int,
  sets:        Int,
  ways:        Int,
  tagBits:     Int,
  dir_init_fn: () => T,
  dir_hit_fn: T => Bool,
  invalid_way_sel: (Seq[T], UInt) => (Bool, UInt),
  replacement: String)(implicit p: Parameters)
    extends MultiIOModule {

  val setBits = log2Ceil(sets)
  val wayBits = log2Ceil(ways)
  val dir_init = dir_init_fn()

  val io = IO(new Bundle() {
    val read = Flipped(DecoupledIO(new Bundle() {
      val tag = UInt(tagBits.W)
      val set = UInt(setBits.W)
      val replacerInfo = new ReplacerInfo()
      val wayMode = Bool()
      val way = UInt(wayBits.W)
    }))
    val resp = ValidIO(new Bundle() {
      val hit = Bool()
      val way = UInt(wayBits.W)
      val tag = UInt(tagBits.W)
      val dir = dir_init.cloneType
      val error = Bool()
    })
    val tag_w = Flipped(DecoupledIO(new Bundle() {
      val tag = UInt(tagBits.W)
      val set = UInt(setBits.W)
      val way = UInt(wayBits.W)
    }))
    val dir_w = Flipped(DecoupledIO(new Bundle() {
      val set = UInt(setBits.W)
      val way = UInt(wayBits.W)
      val dir = dir_init.cloneType
    }))
  })

  val clk_div_by_2 = p(HCCacheParamsKey).sramClkDivBy2
  val resetFinish = RegInit(false.B)
  val resetIdx = RegInit((sets - 1).U)
  // 在Chisel中，SRAMTemplate用于创建一个SRAM，dataType指定SRAM存储的数据类型，nSets指定SRAM中集合的数量，
  // nWays指定每个集合中的路数，singlePort指定SRAM是否为单端口，input_clk_div_by_2指定输入时钟是否需要除2
  val metaArray = Module(new SRAMTemplate(chiselTypeOf(dir_init), sets, ways, singlePort = true, input_clk_div_by_2 = clk_div_by_2))

  // 门控时钟
  val clkGate = Module(new STD_CLKGT_func)
  val clk_en = RegInit(false.B)
  clk_en := ~clk_en
  clkGate.io.TE := false.B
  clkGate.io.E := clk_en
  clkGate.io.CK := clock
  val masked_clock = clkGate.io.Q

  // dirW和tagW使能
  val tag_wen = io.tag_w.valid
  val dir_wen = io.dir_w.valid
  val replacer_wen = RegInit(false.B)
  io.tag_w.ready := true.B
  io.dir_w.ready := true.B
  io.read.ready := !tag_wen && !dir_wen && !replacer_wen && resetFinish

  def tagCode: Code = Code.fromString(p(HCCacheParamsKey).tagECC)

  val eccTagBits = tagCode.width(tagBits)
  val eccBits = eccTagBits - tagBits
  println(s"Tag ECC bits:$eccBits")
  val tagRead = Wire(Vec(ways, UInt(tagBits.W)))
  val eccRead = Wire(Vec(ways, UInt(eccBits.W)))
  val tagArray = Module(new SRAMTemplate(UInt(tagBits.W), sets, ways, singlePort = true, input_clk_div_by_2 = clk_div_by_2))
  // 没有使用ecc，所以eccRead的项都为0
  if(eccBits > 0){
    val eccArray = Module(new SRAMTemplate(UInt(eccBits.W), sets, ways, singlePort = true, input_clk_div_by_2 = clk_div_by_2))
    eccArray.io.w(
      // fire会检查输入信号valid和输出信号ready是否同时为高电平
      // 若是这样，fire自动将输入信号的有效数据传输到输出信号中，并返回高电平
      io.tag_w.fire(),
      tagCode.encode(io.tag_w.bits.tag).head(eccBits),
      io.tag_w.bits.set,
      UIntToOH(io.tag_w.bits.way)
    )
    if (clk_div_by_2) {
      eccArray.clock := masked_clock
    }
    eccRead := eccArray.io.r(io.read.fire(), io.read.bits.set).resp.data
  } else {
    eccRead.foreach(_ := 0.U)
  }

  // 将输入信号io.tag_w的数据写入tagArray中。SRAMWriteBus中有函数定义def apply(valid: Bool, data: T, setIdx: UInt, waymask: UInt): SRAMWriteBus[T]
  // 所以四个信号分别对应有效位、tag数据、写入的地址set、写入的地址way（独热码，每个地址都对应唯一的路号）
  tagArray.io.w(
    io.tag_w.fire(),
    io.tag_w.bits.tag,
    io.tag_w.bits.set,
    UIntToOH(io.tag_w.bits.way)
  )
  // 将tagArray的数据读到tagRead，包含的两位是有效位和地址set，地址way默认等于1
  tagRead := tagArray.io.r(io.read.fire(), io.read.bits.set).resp.data

  // 若时钟二分频，那么使用门控时钟
  if (clk_div_by_2) {
    metaArray.clock := masked_clock
    tagArray.clock := masked_clock
  }

  val reqReg = RegEnable(io.read.bits, enable = io.read.fire())
  val reqValidReg = RegInit(false.B)
  if (clk_div_by_2) {
    reqValidReg := RegNext(io.read.fire())
  } else {
    reqValidReg := io.read.fire()
  }

  val hit_s1 = Wire(Bool())
  val way_s1 = Wire(UInt(wayBits.W))

  // clientDir采用随机替换算法，selfDir采用PLRU算法
  val repl = ReplacementPolicy.fromString(replacement, ways)
  val repl_state = if(replacement == "random"){
    // 如果是随机替换算法，返回0
    when(io.tag_w.fire()){
      repl.miss
    }
    0.U
  } else {
    // 创建一个替换SRAM
    val replacer_sram = Module(new SRAMTemplate(UInt(repl.nBits.W), sets, singlePort = true, shouldReset = true))
    val repl_sram_r = replacer_sram.io.r(io.read.fire(), io.read.bits.set).resp.data(0)
    val repl_state_hold = WireInit(0.U(repl.nBits.W))
    // RegNext将io.read.fire延迟一个时钟周期并存在寄存器中，若该信号为高电平，则保持repl_sram_r不变，否则则将其清零
    repl_state_hold := HoldUnless(repl_sram_r, RegNext(io.read.fire(), false.B))
    val next_state = repl.get_next_state(repl_state_hold, way_s1)
    replacer_sram.io.w(replacer_wen, RegNext(next_state), RegNext(reqReg.set), 1.U)
    repl_state_hold
  }

  // 若是二分频时钟，reqValidReg当io.read有效的下一个时钟周期有效
  // 若不是，则在io.read有效后直接有效
  io.resp.valid := reqValidReg
  // 将metaArray的数据读到metas
  val metas = metaArray.io.r(io.read.fire(), io.read.bits.set).resp.data
  // tagRead是一个向量，这个就是取每一位的低tagBits位与readd的tag进行比较
  val tagMatchVec = tagRead.map(_(tagBits - 1, 0) === reqReg.tag)
  // metas是一个向量，dir_hit_fn是输入函数，猜测应该是判断dir是否命中
  val metaValidVec = metas.map(dir_hit_fn)
  // 将上面两个向量进行与运算，理论上只有一位为1
  val hitVec = tagMatchVec.zip(metaValidVec).map(x => x._1 && x._2)
  val hitWay = OHToUInt(hitVec)
  // 若存在way为INVALID，则选INVALID，否则选替换算法得出的way，有优先级
  val replaceWay = repl.get_replace_way(repl_state)
  val (inv, invalidWay) = invalid_way_sel(metas, replaceWay)
  val chosenWay = Mux(inv, invalidWay, replaceWay)

  /* stage 0: io.read.fire
     stage #: wait for sram
     stage 1: generate hit/way, io.resp.valid = TRUE (will latch into MSHR)
     stage 2: output latched hit/way, output dir/tag
  */
  hit_s1 := Cat(hitVec).orR()
  way_s1 := Mux(reqReg.wayMode, reqReg.way, Mux(hit_s1, hitWay, chosenWay))

  // reqValidReg有效时，hit_s1的值给到hit_s2，否则为false
  val hit_s2 = RegEnable(hit_s1, false.B, reqValidReg)
  val way_s2 = RegEnable(way_s1, 0.U, reqValidReg)
  val metaAll_s2 = RegEnable(metas, reqValidReg)
  val tagAll_s2 = RegEnable(tagRead, reqValidReg)
  val meta_s2 = metaAll_s2(way_s2)
  val tag_s2 = tagAll_s2(way_s2)

  val errorAll_s1 = VecInit(eccRead.zip(tagRead).map{x => tagCode.decode(x._1 ## x._2).error})
  val errorAll_s2 = RegEnable(errorAll_s1, reqValidReg)
  val error_s2 = errorAll_s2(way_s2)

  // resp输出
  io.resp.bits.hit := hit_s2
  io.resp.bits.way := way_s2
  io.resp.bits.dir := meta_s2
  io.resp.bits.tag := tag_s2
  io.resp.bits.error := io.resp.bits.hit && error_s2

  // 从dir_w向metaArray写数据
  metaArray.io.w(
    !resetFinish || dir_wen,
    Mux(resetFinish, io.dir_w.bits.dir, dir_init),
    Mux(resetFinish, io.dir_w.bits.set, resetIdx),
    // Fill表示将ways个true值复制到一个数组
    Mux(resetFinish, UIntToOH(io.dir_w.bits.way), Fill(ways, true.B))
  )

  // 定义了一个计数范围为2的计数器
  val cycleCnt = Counter(true.B, 2)
  val resetMask = if (clk_div_by_2) cycleCnt._1(0) else true.B
  when(resetIdx === 0.U && resetMask) {
    resetFinish := true.B
  }
  when(!resetFinish && resetMask) {
    resetIdx := resetIdx - 1.U
  }

}

trait HasUpdate {
  def doUpdate(info: ReplacerInfo): Bool
}

trait UpdateOnRelease extends HasUpdate {
  override def doUpdate(info: ReplacerInfo) = {
    info.channel(2) && info.opcode === TLMessages.ReleaseData
  }
}

trait UpdateOnAcquire extends HasUpdate {
  override def doUpdate(info: ReplacerInfo) = {
    info.channel(0) && (info.opcode === TLMessages.AcquirePerm || info.opcode === TLMessages.AcquireBlock)
  }
}

abstract class SubDirectoryDoUpdate[T <: Data](
  wports:      Int,
  sets:        Int,
  ways:        Int,
  tagBits:     Int,
  dir_init_fn: () => T,
  dir_hit_fn:  T => Bool,
  invalid_way_sel: (Seq[T], UInt) => (Bool, UInt),
  replacement: String)(implicit p: Parameters)
    extends SubDirectory[T](
      wports, sets, ways, tagBits,
      dir_init_fn, dir_hit_fn, invalid_way_sel,
      replacement
    ) with HasUpdate {

  val update = doUpdate(reqReg.replacerInfo)
  when(reqValidReg && update) {
    replacer_wen := true.B
  }.otherwise {
    replacer_wen := false.B
  }
}
