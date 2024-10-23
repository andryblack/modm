/*
 * Copyright (c) 2022, Andrey Kunitsyn
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once

#include <array>
#include <cstdint>
#include <cassert>

namespace modm::platform::pio
{

	struct Instruction {
		uint16_t value;
		constexpr Instruction(uint16_t val) : value(val) {}
		constexpr uint16_t store(size_t offset) const {
			return ((value & 0xe000) == 0x0000) ? (value + offset) : value;
		}
		constexpr Instruction operator | (uint16_t params) const {
			return Instruction(value | params);
		}
		constexpr bool operator == (uint16_t val) const {
			return value == val;
		}
	};

	enum class Opcode : uint16_t {
		JMP 	= 0x0000 << 13,
		WAIT 	= 0x0001 << 13,
		IN 		= 0x0002 << 13,
		OUT 	= 0x0003 << 13,
		PUSH 	= 0x0004 << 13,
		PULL 	= (0x0004 << 13) | (0x0001 << 7),
		MOV 	= 0x0005 << 13,
		IRQ 	= 0x0006 << 13,
		SET 	= 0x0007 << 13,
	};

	struct OptionalValue {
		uint16_t value{0};
		bool is_set{false};
	};

	static constexpr uint16_t NO_SIDE_VALUE = 0xffff;

	template <uint16_t side = NO_SIDE_VALUE,uint16_t delay = 0>
	struct SideDelay {
		static constexpr const uint16_t side_value = (side == NO_SIDE_VALUE) ? 0 : side;
		static constexpr const bool side_value_present = side != NO_SIDE_VALUE;
		static constexpr const uint16_t delay_value = delay;
	};

	template <typename Res>
	struct InstructionEncoding {
		static constexpr const Opcode opcode = Res::opcode;
		static constexpr const uint16_t DELAY_SIDESET_SHIFT = 8;
		static constexpr const uint16_t DELAY_SIDESET_MASK = 0x1f << DELAY_SIDESET_SHIFT;
		static constexpr const uint16_t ARG1_SHIFT = 5;

		using SideDelayConfig = SideDelay<>;

		template <uint16_t side = 0>
		struct SideValue : Res {
			using SideDelayConfig = SideDelay<side,0>;
			constexpr SideValue() {}
			template <uint16_t delay = 0>
			struct DelayValue : Res {
				constexpr DelayValue() {}
				using SideDelayConfig = SideDelay<side,delay>;
				template <typename Encoder>
				constexpr Instruction encode(const Encoder& coder) const {
					return Res::encode(coder) | (coder.template encodeSideDelay<SideDelayConfig>() << DELAY_SIDESET_SHIFT);
				}
			};
			template <uint16_t val>
			constexpr auto delay() {
				return DelayValue<val>();
			}
			template <typename Encoder>
			constexpr Instruction encode(const Encoder& coder) const {
				return Res::encode(coder) | (coder.template encodeSideDelay<SideDelayConfig>() << DELAY_SIDESET_SHIFT);
			}
		};
		template <uint16_t delay = 0>
		struct DelayValue : Res {
			constexpr DelayValue() {}
			using SideDelayConfig = SideDelay<NO_SIDE_VALUE,delay>;
			template <typename Encoder>
			constexpr Instruction encode(const Encoder& coder) const {
				return Res::encode(coder) | (coder.template encodeSideDelay<SideDelayConfig>() << DELAY_SIDESET_SHIFT);
			}
		};

		constexpr InstructionEncoding()  {}
		template <uint16_t val>
		constexpr auto side() const {
			return SideValue<val>();
		}
		template <uint16_t val>
		constexpr auto delay() {
			return DelayValue<val>();
		}
		template <typename Encoder>
		constexpr Instruction encode(const Encoder& ) const {
			return Instruction(uint16_t(opcode));
		}
	};

	enum class JmpCondition : uint16_t {
		ALWAYS 	= 0x00,
		NOT_X 	= 0x01,
		X_DEC 	= 0x02,
		NOT_Y	= 0x03,
		Y_DEC 	= 0x04,
		X_NE_Y	= 0x05,
		PIN 	= 0x06,
		NOT_ORSE = 0x07,
	};
	template <typename Label = void,JmpCondition condition = JmpCondition::ALWAYS,uint16_t addr = 0x0000>
	struct Jmp : InstructionEncoding<Jmp<Label,condition,addr>> {
		static constexpr const Opcode opcode = Opcode::JMP;
		static constexpr const JmpCondition condition_value = condition;
		static constexpr const uint16_t addr_value = addr;
		static_assert(addr_value < 0x0f, "Invalid addr");

		using LabelName = Label;
		using Base = InstructionEncoding<Jmp<Label,condition,addr>>;

		explicit constexpr Jmp() {}
		static constexpr auto not_x() { return Jmp<LabelName,JmpCondition::NOT_X,addr_value>(); }
		static constexpr auto x_dec() { return Jmp<LabelName,JmpCondition::X_DEC,addr_value>(); }
		static constexpr auto not_y() { return Jmp<LabelName,JmpCondition::NOT_Y,addr_value>(); }
		static constexpr auto y_dec() { return Jmp<LabelName,JmpCondition::Y_DEC,addr_value>(); }
		static constexpr auto x_ne_y() { return Jmp<LabelName,JmpCondition::X_NE_Y,addr_value>(); }
		static constexpr auto pin() { return Jmp<LabelName,JmpCondition::PIN,addr_value>(); }
		static constexpr auto not_osre() { return Jmp<LabelName,JmpCondition::NOT_ORSE,addr_value>(); }

		template <typename ToLabel>
		static constexpr auto to() { return Jmp<ToLabel,condition_value,addr_value>(); }

		template <typename Encoder>
		static constexpr uint16_t getAddr(const Encoder& coder) {
			if constexpr (std::is_same_v<Label,void>) {
				return addr_value;
			} else {
				return coder.template getOffset<Label>();
			}
		}

		template <typename Encoder>
		constexpr Instruction encode(const Encoder& coder) const {
			return Base::encode(coder) | (uint16_t(condition_value) << Base::ARG1_SHIFT) | getAddr(coder);
		}

	};

	enum class WaitSource : uint16_t {
		GPIO 		= 0x00,
		PIN 		= 0x01,
		IRQ 		= 0x02
	};
	template <int polarity>
	struct Wait  {
		static_assert(polarity == 0 || polarity == 1, "Invalid polarity");

		// @todo relative syntax
		template <WaitSource source,uint16_t index>
		struct WaitImpl : InstructionEncoding<WaitImpl<source,index>> {
			static constexpr const Opcode opcode = Opcode::WAIT;
			using Base = InstructionEncoding<WaitImpl<source,index>>;
			static constexpr WaitSource source_value = source;
			static constexpr uint16_t polarity_value = polarity;
			static constexpr uint16_t index_value = index;
			explicit constexpr WaitImpl() {}
			template <typename Encoder>
			constexpr Instruction encode(const Encoder& coder) const {
				return Base::encode(coder) |
					(polarity_value << 7) |
					(uint16_t(source_value) << Base::ARG1_SHIFT) |
					index_value;
			}
		};

		explicit constexpr Wait() {}
		template <uint16_t gpio_num>
		static constexpr auto gpio() { return WaitImpl<WaitSource::GPIO,gpio_num>(); }
		template <uint16_t pin_num>
		static constexpr auto pin() { return WaitImpl<WaitSource::PIN,pin_num>(); }
		template <uint16_t irq_num>
		static constexpr auto irq() { return WaitImpl<WaitSource::PIN,irq_num>(); }
	};

	enum class InSource : uint16_t {
		PINS 		= 0x00,
		X 			= 0x01,
		Y 			= 0x02,
		NULL_T 		= 0x03,
		ISR 		= 0x06,
		OSR 		= 0x07,
	};
	struct In {
		template <InSource source,uint16_t bits>
		struct InImpl : InstructionEncoding<InImpl<source,bits>> {
			static_assert(bits >= 1 && bits <= 32,"Invalid bits count");
			static constexpr const Opcode opcode = Opcode::IN;
			using Base = InstructionEncoding<InImpl<source,bits>>;
			static constexpr InSource source_value = source;
			static constexpr uint16_t bits_value = bits == 32 ? 0 : bits;
			explicit constexpr InImpl() {}
			template <typename Encoder>
			constexpr Instruction encode(const Encoder& coder) const {
				return Base::encode(coder) |
					(uint16_t(source_value) << Base::ARG1_SHIFT) |
					bits_value;
			}
		};
		explicit constexpr In() {}
	};
	enum class OutDestination : uint16_t {
		PINS 	= 0x00,
		X 		= 0x01,
		Y 		= 0x02,
		NULL_T 	= 0x03,
		PINDIRS = 0x04,
		PC 		= 0x05,
		ISR 	= 0x06,
		EXEC 	= 0x07,
	};

	struct Out {
		template <OutDestination destination,uint16_t bits>
		struct OutImpl : InstructionEncoding<OutImpl<destination,bits>> {
			using Base = InstructionEncoding<OutImpl<destination,bits>>;
			static_assert(bits >= 1 && bits <= 32,"Invalid bits count");
			static constexpr const Opcode opcode = Opcode::OUT;
			static constexpr OutDestination destination_value = destination;
			static constexpr uint16_t bits_value = bits == 32 ? 0 : bits;
			constexpr OutImpl() {}
			template <typename Encoder>
			constexpr Instruction encode(const Encoder& coder) const {
				return Base::encode(coder) |
					(uint16_t(destination_value) << Base::ARG1_SHIFT) |
					bits_value;
			}
		};

		template <uint16_t nbits>
		constexpr auto pins() { return OutImpl<OutDestination::PINS,nbits>(); }
		template <uint16_t nbits>
		constexpr auto x() { return OutImpl<OutDestination::X,nbits>(); }
		template <uint16_t nbits>
		constexpr auto y() { return OutImpl<OutDestination::Y,nbits>(); }
		template <uint16_t nbits>
		constexpr auto null() { return OutImpl<OutDestination::NULL_T,nbits>(); }
		template <uint16_t nbits>
		constexpr auto pindirs() { return OutImpl<OutDestination::PINDIRS,nbits>(); }
		template <uint16_t nbits>
		constexpr auto pc() { return OutImpl<OutDestination::PC,nbits>(); }
		template <uint16_t nbits>
		constexpr auto isr() { return OutImpl<OutDestination::ISR,nbits>(); }
		template <uint16_t nbits>
		constexpr auto exec() { return OutImpl<OutDestination::EXEC,nbits>(); }

	};

	template <bool IfFull = false, bool Block = true>
	struct Push : InstructionEncoding<Push<IfFull,Block>> {
		static constexpr const Opcode opcode = Opcode::PUSH;
		using Base = InstructionEncoding<Push<IfFull,Block>>;
		static constexpr const bool iffull_value = IfFull;
		static constexpr const bool block_value = Block;
		constexpr Push() {}
		constexpr auto iffull() { return Push<true,block_value>(); }
		constexpr auto block() { return Push<iffull_value,true>(); }
		constexpr auto noblock() { return Push<iffull_value,false>(); }
		template <typename Encoder>
		constexpr Instruction encode(const Encoder& coder) const {
			return Base::encode(coder) | (iffull_value ? (1<<6) : 0) | (block_value ? (1<<5) : 0);
		}
	};
	template <bool IfEmpty = false, bool Block = true>
	struct Pull : InstructionEncoding<Pull<IfEmpty,Block>> {
		static constexpr const Opcode opcode = Opcode::PULL;
		using Base = InstructionEncoding<Pull<IfEmpty,Block>>;
		static constexpr const bool ifempty_value = IfEmpty;
		static constexpr const bool block_value = Block;
		constexpr Pull()  {}
		constexpr auto ifempty() { return Pull<true,block_value>(); }
		constexpr auto block() { return Pull<ifempty_value,true>(); }
		constexpr auto noblock() { return Pull<ifempty_value,false>(); }
		template <typename Encoder>
		constexpr Instruction encode(const Encoder& coder) const {
			return Base::encode(coder) | (ifempty_value ? (1<<6) : 0) | (block_value ? (1<<5) : 0);
		}
	};

	enum class MovDestination : uint16_t {
		PINS 	= 0x00,
		X 		= 0x01,
		Y 		= 0x02,
		EXEC 	= 0x04,
		PC 		= 0x05,
		ISR 	= 0x06,
		OSR 	= 0x07,
	};

	enum class MovSource : uint16_t {
		PINS 	= 0x00,
		X 		= 0x01,
		Y 		= 0x02,
		NULL_T 	= 0x03,
		STATUS 	= 0x05,
		ISR 	= 0x06,
		OSR 	= 0x07,
	};

	enum class MovOperation : uint16_t {
		NONE 	= 0x00,
		INVERT 	= 0x01,
		REVERSE = 0x02,
	};

	struct Mov {
		template <MovDestination destination,MovOperation operation = MovOperation::NONE>
		struct MovDst  {
			template <MovSource source>
			struct MovImpl : InstructionEncoding<MovImpl<source>> {
				static constexpr const Opcode opcode = Opcode::MOV;
				static constexpr MovSource source_value = source;
				static constexpr MovDestination destination_value = destination;
				static constexpr MovOperation operation_value = operation;

				static constexpr size_t OP_SHIFT = 3;
				static constexpr size_t DESTINATION_SHIFT = 5;
				template <typename Encoder>
				constexpr Instruction encode(const Encoder& coder) const {
					return InstructionEncoding<MovImpl<source>>::encode(coder) | uint16_t(source_value) |
						(uint16_t(operation_value)<<OP_SHIFT) |
						(uint16_t(destination_value) << DESTINATION_SHIFT);
				}
			};

			static constexpr auto invert() { return MovDst<destination,MovOperation::INVERT>(); }
			static constexpr auto reverse() { return MovDst<destination,MovOperation::REVERSE>(); }

			static  constexpr auto pins() { return MovImpl<MovSource::PINS>(); }
			static  constexpr auto x() { return MovImpl<MovSource::X>(); }
			static  constexpr auto y() { return MovImpl<MovSource::Y>(); }
			static  constexpr auto null() { return MovImpl<MovSource::NULL_T>(); }
			static  constexpr auto status() { return MovImpl<MovSource::STATUS>(); }
			static  constexpr auto isr() { return MovImpl<MovSource::ISR>(); }
			static  constexpr auto osr() { return MovImpl<MovSource::OSR>(); }
		};

		static constexpr auto pins()  { return MovDst<MovDestination::PINS>(); }
		static constexpr auto x()  { return MovDst<MovDestination::X>(); }
		static constexpr auto y()  { return MovDst<MovDestination::Y>(); }
		static constexpr auto exec()  { return MovDst<MovDestination::EXEC>(); }
		static constexpr auto pc()  { return MovDst<MovDestination::PC>(); }
		static constexpr auto isr()  { return MovDst<MovDestination::ISR>(); }
		static constexpr auto osr()  { return MovDst<MovDestination::OSR>(); }
	};

	struct Irq {
		// @todo relative syntax
		template <bool clear,bool wait,uint16_t index>
		struct IrqImpl : InstructionEncoding<IrqImpl<clear,wait,index>> {
			static constexpr const Opcode opcode = Opcode::IRQ;
			using Base = InstructionEncoding<IrqImpl<clear,wait,index>>;
			static constexpr bool clear_value = clear;
			static constexpr bool wait_value = wait;
			static constexpr uint16_t index_value = index;
			explicit constexpr IrqImpl() {}
			template <typename Encoder>
			constexpr Instruction encode(const Encoder& coder) const {
				return Base::encode(coder) |
					(clear_value ? (1<<6) : 0) |
					(wait_value ? (1<<5) : 0) |
					index_value;
			}
		};
		constexpr Irq() {}
		template <uint16_t index>
		static constexpr auto set() { return IrqImpl<false,false,index>(); }
		template <uint16_t index>
		static constexpr auto nowait() { return IrqImpl<false,false,index>(); }
		template <uint16_t index>
		static constexpr auto wait() { return IrqImpl<false,true,index>(); }
		template <uint16_t index>
		static constexpr auto clear() { return IrqImpl<true,false,index>(); }
	};

	enum class SetDestination : uint16_t{
		PINS 	= 0x00,
		X 		= 0x01,
		Y 		= 0x02,
		PINDIRS = 0x04,
	};

	struct Set {
		template <SetDestination destination,uint16_t value>
		struct SetImpl : InstructionEncoding<SetImpl<destination,value>> {
			static_assert(value <= 31,"invalid data value");
			static constexpr const Opcode opcode = Opcode::SET;
			static constexpr SetDestination destination_value = destination;
			static constexpr uint16_t data_value = value;
			static constexpr size_t DESTINATION_SHIFT = 5;
			template <typename Encoder>
			constexpr Instruction encode(const Encoder& coder) const {
				return InstructionEncoding<SetImpl<destination,value>>::encode(coder) | data_value |
					(uint16_t(destination_value) << DESTINATION_SHIFT);
			}
		};

		constexpr Set()  {}
		template <size_t data>
		static constexpr auto pins() { return SetImpl<SetDestination::PINS,data>(); }
		template <size_t data>
		static constexpr auto x() { return SetImpl<SetDestination::X,data>(); }
		template <size_t data>
		static constexpr auto y() { return SetImpl<SetDestination::Y,data>(); }
		template <size_t data>
		static constexpr auto pindirs() { return SetImpl<SetDestination::PINDIRS,data>(); }

	};

	static inline constexpr auto Nop() {
		return Mov().y().y();
	}

	template <typename ...Labels>
	struct LabelsList;

	template <typename Settings,typename Labels,typename Instructions>
	struct ProgramBuilder;

	template <typename Name,size_t Offset>
	struct Label {
		using name = Name;
		static constexpr size_t offset = Offset;
	};

	template <>
	struct LabelsList<> {
		template <typename Name,size_t offset>
		struct add_s {
			using result = LabelsList<Label<Name,offset> >;
		};
		template <typename Name,size_t offset>
		using add = typename add_s<Name,offset>::result;

		template <typename Name>
		static constexpr bool exist() {
			return false;
		}
	};

	template <typename FirstLabel,typename ...Labels>
	struct LabelsList<FirstLabel,Labels...> {
		template <typename Name>
		static constexpr size_t find(){
			if constexpr (std::is_same_v<Name,typename FirstLabel::name>) {
				return FirstLabel::offset;
			} else {
				static_assert(sizeof ...(Labels)!=0,"Not found label");
				return LabelsList<Labels...>::template find<Name>();
			}
		}
		template <typename Name>
		static constexpr bool exist() {
			if constexpr(std::is_same_v<Name,typename FirstLabel::name>) {
				return true;
			} else {
				return LabelsList<Labels...>::template exist<Name>();
			}
		}
		template <typename Name,size_t offset>
		struct add_s {
			static_assert(!exist<Name>(),"Already added label");
			using result = LabelsList<FirstLabel,Labels...,Label<Name,offset> >;
		};
		template <typename Name,size_t offset>
		using add = typename add_s<Name,offset>::result;
	};


	template <typename ...Instructions>
	struct InstructionsList;

	template <>
	struct InstructionsList<> {
		static constexpr size_t length = 0;
		explicit constexpr InstructionsList() {}
		template <typename Instruction>
		constexpr InstructionsList<Instruction> add( Instruction&& instr ) const {
			return InstructionsList<Instruction>(std::move(instr),*this);
		}
	};


	template <typename FirstInstruction,typename ...Instructions>
	struct InstructionsList<FirstInstruction,Instructions...> {
		static constexpr size_t length = 1 + InstructionsList<Instructions...>::length;
		FirstInstruction head;
		InstructionsList<Instructions...> tail;
		explicit constexpr InstructionsList(FirstInstruction&& inst,const InstructionsList<Instructions...>& tail) : head(std::move(inst)),tail(tail) {}
		template <typename Instruction>
		constexpr InstructionsList<Instruction,FirstInstruction,Instructions...> add( Instruction&& instr) const {
			return InstructionsList<Instruction,FirstInstruction,Instructions...>(std::move(instr),*this);
		}
		template <size_t I,typename ProgramBuilder>
		constexpr Instruction encode_i(const ProgramBuilder& builder) const {
			if constexpr (I == length-1) {
				return head.encode(builder);
			} else {
				return tail.template encode_i<I>(builder);
			}
		}

		template <typename ProgramBuilder,std::size_t... I>
		constexpr std::array<Instruction,length> build_i(const ProgramBuilder& builder,std::index_sequence<I...>) const {
			return {{ encode_i<I>(builder)..., }};
		};
		template <typename ProgramBuilder>
		constexpr std::array<Instruction,length> build(const ProgramBuilder& builder) const {
			using indexes = std::make_index_sequence<length>;
			return build_i(builder,indexes{});
		};
	};


	template <size_t Length,typename Labels>
	struct ProgramCode  {
		static constexpr const size_t length = Length;
		const uint16_t wrap_target;
		const uint16_t wrap;
		constexpr ProgramCode( std::array<Instruction,Length>&& code,uint16_t wrap_target,uint16_t wrap) : wrap_target(wrap_target),wrap(wrap),code(std::move(code)) {

		}
		const std::array<Instruction,Length> code;
		template <typename Label>
		static constexpr size_t getOffset() {
			return Labels::template find<Label>();
		}
	};


	template <uint32_t sideset_count_t = 0,bool sideset_is_opt_t = true, uint16_t delay_max_t = 31, uint16_t sideset_max_t = 0>
	struct ProgramSettings {
		static constexpr uint32_t sideset_count = sideset_count_t;
		static constexpr bool sideset_is_opt = sideset_is_opt_t;
		static constexpr uint16_t delay_max = delay_max_t;
		static constexpr uint16_t sideset_max = sideset_max_t;
	};
	template <size_t count>
	struct ProgramSettingsSideset {
		static_assert(count <= 5,"maximum number of side set bits is 5");
		using result = ProgramSettings<count,false,(1u << (5 - count)) - 1,(1u << count) - 1>;
	};
	template <size_t count>
	struct ProgramSettingsOptSideset {
		static_assert(count <= 4,"maximum number of side set bits is 4");
		using result = ProgramSettings<count+1,true,(1u << (5 - (count+1))) - 1,(1u << count) - 1>;
	};

	struct WrapSettings {
		OptionalValue wrap_target;
		OptionalValue wrap_instr;
	};

	template <typename SettingsConfig, typename Labels,typename Instructions>
	struct ProgramBuilder {
		using labels = Labels;
		using Settings = SettingsConfig;
		Instructions instructions;
		WrapSettings wrap_settings;

		explicit constexpr ProgramBuilder() {}
		explicit constexpr ProgramBuilder(Instructions && instructions,const WrapSettings& wrap_settings) : instructions(std::move(instructions)), wrap_settings(wrap_settings) {}
		explicit constexpr ProgramBuilder(const Instructions & instructions,const WrapSettings& wrap_settings) : instructions(instructions), wrap_settings(wrap_settings) {}

		template <size_t count>
		constexpr auto sideset() const {
			return ProgramBuilder<typename ProgramSettingsSideset<count>::result,Labels,Instructions>(instructions,wrap_settings);
		}
		template <size_t count>
		constexpr auto sideset_opt() const {
			return ProgramBuilder<typename ProgramSettingsOptSideset<count>::result,Labels,Instructions>(instructions,wrap_settings);
		}

		template <typename SideDelayConfig>
		static constexpr uint16_t encodeSideDelay() {
			auto res = SideDelayConfig::delay_value;
			static_assert(SideDelayConfig::delay_value <= Settings::delay_max,"delay too long");

			if constexpr (SideDelayConfig::side_value_present) {
				static_assert(SideDelayConfig::side_value<=Settings::sideset_max, "side value overflow");
				res |= (SideDelayConfig::side_value << (5-Settings::sideset_count));
				if constexpr (Settings::sideset_is_opt) {
					res |= 1 << 4;
				}
			} else {
				static_assert(Settings::sideset_is_opt, "sideset is not optional");
			}
			//static_assert(side < (1<<sideset_count),"side bits overflow");
			return res;
		}
		template <typename Label>
		static constexpr size_t getOffset() {
			return Labels::template find<Label>();
		}
		constexpr ProgramBuilder& wrap() {
			wrap_settings.wrap_instr.is_set = true;
			//static_assert(Instructions::length>0 && ".wrap cannot be placed before the first program instruction");
			wrap_settings.wrap_instr.value = Instructions::length-1;
			return *this;
		}
		constexpr ProgramBuilder& wrapTarget() {
			//static_assert(!settings.wrap_target.is_set && ".wrap_target only once");
			wrap_settings.wrap_target.value = Instructions::length;
			wrap_settings.wrap_target.is_set = true;
			return *this;
		}
		template <typename LabelName>
		constexpr auto label(const LabelName& = LabelName{}) const {
			return ProgramBuilder<Settings,typename Labels::template add<LabelName,Instructions::length>,Instructions>(instructions,wrap_settings);
		}
		template <typename Instr>
		constexpr auto instr( Instr&& instr ) const {
			auto added_instructions = instructions.add(std::move(instr));
			using AddedInstructions = decltype(added_instructions);
			return ProgramBuilder<Settings,Labels,AddedInstructions>(std::move(added_instructions),wrap_settings);
		}
		constexpr ProgramCode<Instructions::length,Labels> end() const {
			return ProgramCode<Instructions::length,Labels>(instructions.build(*this),
				wrap_settings.wrap_target.is_set?wrap_settings.wrap_target.value:0,
				wrap_settings.wrap_instr.is_set?wrap_settings.wrap_instr.value:(Instructions::length-1));
		}
	};


	using EmptyProgram = ProgramBuilder<ProgramSettings<>,LabelsList<>,InstructionsList<>>;
	static inline constexpr EmptyProgram  ProgramBegin() {
		return EmptyProgram();
	};
}
