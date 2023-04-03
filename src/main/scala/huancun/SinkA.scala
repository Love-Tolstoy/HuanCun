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
import freechips.rocketchip.tilelink._

class SinkA(implicit p: Parameters) extends HuanCunModule {
  val io = IO(new Bundle() {
    val a = Flipped(DecoupledIO(new TLBundleA(edgeIn.bundle)))
    val alloc = DecoupledIO(new MSHRRequest)
    val task = Flipped(DecoupledIO(new SinkAReq))
    // SourceD
    val d_pb_pop = Flipped(DecoupledIO(new PutBufferPop))
    val d_pb_beat = Output(new PutBufferBeatEntry)
    // SourceA
    val a_pb_pop = Flipped(DecoupledIO(new PutBufferPop))
    val a_pb_beat = Output(new PutBufferBeatEntry)
  })

  // TODO: Does task for SinkA necessary?
  io.task.ready := false.B

  val a = io.a
  // count统计当a有效时，统计其中true的数量
  // 把输入的数据拆成四份
  val (first, last, done, count) = edgeIn.count(a)
  // 接收到的请求是否携带数据
  val hasData = edgeIn.hasData(a.bits)

  // beats = 2? 每一个PutBufferBeatEntry项包含256位宽的数据和32位宽的掩码（1位掩码对应位数据）
  // bufBlocks = mshrs / 2 = 7，所以purBuffer是一个7*2，每一项都是PutBufferBeatEntry的存储器
  val beats = blockBytes / beatBytes
  val putBuffer = Reg(Vec(bufBlocks, Vec(beats, new PutBufferBeatEntry())))
  // 变量beatVals来记录每一项是否有效，初始化值为false.B
  val beatVals = RegInit(VecInit(Seq.fill(bufBlocks) {
    VecInit(Seq.fill(beats) { false.B })
  }))
  // 括号内的操作对bufBlocks行内的beats个元素进行或规约，然后组成一个7位的向量，最后转换为7位的UInt类型
  // 它的每一位标志着对应的bufBlocks内是否有beat是有效
  val bufVals = VecInit(beatVals.map(_.asUInt().orR())).asUInt()
  // 只有bufVals的所有位都为1才把满标志位置1，但是一个bufBlocks中只有一个有效也会置1
  val full = bufVals.andR()
  // 当满了且请求带数据
  val noSpace = full && hasData
  // 对bufVals按位取反，PriorityEncoder作用是返回输入向量的最低位为1的位置
  // PriorityEncoder("b0110".U) // results in 1.U
  // insertIdx标志最低位的能够容纳数据的某个数据行
  val insertIdx = PriorityEncoder(~bufVals)
  // 请求有效且接受到了第一个数据位
  val insertIdxReg = RegEnable(insertIdx, a.fire() && first)

  // 当请求有效且携带数据，当接收到的是首个数据，那么直接赋值，否则使用寄存器的值
  when(a.fire() && hasData) {
    when(first) {
      putBuffer(insertIdx)(count).data := a.bits.data
      putBuffer(insertIdx)(count).mask := a.bits.mask
      putBuffer(insertIdx)(count).corrupt := a.bits.corrupt
      beatVals(insertIdx)(count) := true.B
    }.otherwise({
      putBuffer(insertIdxReg)(count).data := a.bits.data
      putBuffer(insertIdxReg)(count).mask := a.bits.mask
      putBuffer(insertIdxReg)(count).corrupt := a.bits.corrupt
      beatVals(insertIdxReg)(count) := true.B
    })
  }

  // 为什么检查索引0
  val bufferLeakCnt = RegInit(0.U(12.W)) // check buffer leak for index 0
  dontTouch(bufferLeakCnt)
  when(bufVals(0)) {
    bufferLeakCnt := bufferLeakCnt + 1.U
  }.otherwise {
    bufferLeakCnt := 0.U
  }

  when(bufferLeakCnt === 800.U) {
    assert(false.B, "buffer leak at index 0")
  }

  // 将地址刨去bankset，然后分块
  val (tag, set, offset) = parseAddress(a.bits.address)

  // 当输入请求有效，且接到第一个数据位，并且没有发生请求阻塞
  io.alloc.valid := a.valid && first && !noSpace
  a.ready := Mux(first, io.alloc.ready && !noSpace, true.B)

  val allocInfo = io.alloc.bits
  allocInfo.channel := 1.U(3.W)
  allocInfo.opcode := a.bits.opcode
  allocInfo.param := a.bits.param
  allocInfo.size := a.bits.size
  allocInfo.source := a.bits.source
  allocInfo.set := set
  allocInfo.tag := tag
  allocInfo.off := offset
  allocInfo.mask := a.bits.mask
  allocInfo.bufIdx := insertIdx
  // 预取？ lift用于从Option类型中提取值，如果该字段不存在，则返回None
  allocInfo.needHint.foreach(_ := a.bits.user.lift(PrefetchKey).getOrElse(false.B))
  allocInfo.isPrefetch.foreach(_ := a.bits.opcode === TLMessages.Hint)
  allocInfo.isBop.foreach(_ := false.B)
  allocInfo.alias.foreach(_ := a.bits.user.lift(AliasKey).getOrElse(0.U))
  // allocInfo.preferCache := Mux((a.bits.opcode === TLMessages.Get || a.bits.opcode(2,1) === 0.U), true.B, a.bits.user.lift(PreferCacheKey).getOrElse(true.B))
  if (cacheParams.level == 2) {
    allocInfo.preferCache := a.bits.user.lift(PreferCacheKey).getOrElse(true.B)
  } else {
    allocInfo.preferCache := Mux((a.bits.opcode === TLMessages.Get || a.bits.opcode(2,1) === 0.U), true.B, a.bits.user.lift(PreferCacheKey).getOrElse(true.B))
  }
  allocInfo.dirty := false.B // ignored
  allocInfo.fromProbeHelper := false.B
  allocInfo.fromCmoHelper := false.B
  allocInfo.needProbeAckData.foreach(_ := false.B)

  io.d_pb_pop.ready := beatVals(io.d_pb_pop.bits.bufIdx)(io.d_pb_pop.bits.count)
  // 当d_pb_pop有效时，从PutBuffer取出数据到d_pb_beat，在d_pb_pop有效且数据位最后一位，将对应的元素无效化
  io.d_pb_beat := RegEnable(putBuffer(io.d_pb_pop.bits.bufIdx)(io.d_pb_pop.bits.count), io.d_pb_pop.fire())
  when(io.d_pb_pop.fire() && io.d_pb_pop.bits.last) {
    beatVals(io.d_pb_pop.bits.bufIdx).foreach(_ := false.B)
  }

  io.a_pb_pop.ready := beatVals(io.a_pb_pop.bits.bufIdx)(io.a_pb_pop.bits.count)
  io.a_pb_beat := RegEnable(putBuffer(io.a_pb_pop.bits.bufIdx)(io.a_pb_pop.bits.count), io.a_pb_pop.fire())
  when(io.a_pb_pop.fire() && io.a_pb_pop.bits.last) {
    beatVals(io.a_pb_pop.bits.bufIdx).foreach(_ := false.B)
  }
}
