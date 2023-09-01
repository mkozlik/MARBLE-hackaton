# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: acoustic_transmission.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import labust_pb2 as labust__pb2
import std_pb2 as std__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='acoustic_transmission.proto',
  package='acoustictransmission',
  syntax='proto3',
  serialized_options=None,
  serialized_pb=_b('\n\x1b\x61\x63oustic_transmission.proto\x12\x14\x61\x63oustictransmission\x1a\x0clabust.proto\x1a\tstd.proto\"!\n\x0e\x43ommandRequest\x12\x0f\n\x07\x61\x64\x64ress\x18\x01 \x01(\t\"\x91\x01\n\x10\x41\x63ousticResponse\x12\x0f\n\x07\x61\x64\x64ress\x18\x01 \x01(\t\x12\x0f\n\x07success\x18\x02 \x01(\x08\x12+\n\x07payload\x18\x03 \x01(\x0b\x32\x18.labust.NanomodemPayloadH\x00\x12\'\n\x05range\x18\x04 \x01(\x0b\x32\x16.labust.NanomodemRangeH\x00\x42\x05\n\x03msg\"M\n\x0f\x41\x63ousticRequest\x12\x0f\n\x07success\x18\x01 \x01(\x08\x12)\n\x07request\x18\x02 \x01(\x0b\x32\x18.labust.NanomodemRequest2\xd0\x01\n\x14\x41\x63ousticTransmission\x12i\n\x16StreamAcousticRequests\x12$.acoustictransmission.CommandRequest\x1a%.acoustictransmission.AcousticRequest\"\x00\x30\x01\x12M\n\x15ReturnAcousticPayload\x12&.acoustictransmission.AcousticResponse\x1a\n.std.Empty\"\x00\x62\x06proto3')
  ,
  dependencies=[labust__pb2.DESCRIPTOR,std__pb2.DESCRIPTOR,])




_COMMANDREQUEST = _descriptor.Descriptor(
  name='CommandRequest',
  full_name='acoustictransmission.CommandRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='address', full_name='acoustictransmission.CommandRequest.address', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=78,
  serialized_end=111,
)


_ACOUSTICRESPONSE = _descriptor.Descriptor(
  name='AcousticResponse',
  full_name='acoustictransmission.AcousticResponse',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='address', full_name='acoustictransmission.AcousticResponse.address', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='success', full_name='acoustictransmission.AcousticResponse.success', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='payload', full_name='acoustictransmission.AcousticResponse.payload', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='range', full_name='acoustictransmission.AcousticResponse.range', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
    _descriptor.OneofDescriptor(
      name='msg', full_name='acoustictransmission.AcousticResponse.msg',
      index=0, containing_type=None, fields=[]),
  ],
  serialized_start=114,
  serialized_end=259,
)


_ACOUSTICREQUEST = _descriptor.Descriptor(
  name='AcousticRequest',
  full_name='acoustictransmission.AcousticRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='success', full_name='acoustictransmission.AcousticRequest.success', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='request', full_name='acoustictransmission.AcousticRequest.request', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=261,
  serialized_end=338,
)

_ACOUSTICRESPONSE.fields_by_name['payload'].message_type = labust__pb2._NANOMODEMPAYLOAD
_ACOUSTICRESPONSE.fields_by_name['range'].message_type = labust__pb2._NANOMODEMRANGE
_ACOUSTICRESPONSE.oneofs_by_name['msg'].fields.append(
  _ACOUSTICRESPONSE.fields_by_name['payload'])
_ACOUSTICRESPONSE.fields_by_name['payload'].containing_oneof = _ACOUSTICRESPONSE.oneofs_by_name['msg']
_ACOUSTICRESPONSE.oneofs_by_name['msg'].fields.append(
  _ACOUSTICRESPONSE.fields_by_name['range'])
_ACOUSTICRESPONSE.fields_by_name['range'].containing_oneof = _ACOUSTICRESPONSE.oneofs_by_name['msg']
_ACOUSTICREQUEST.fields_by_name['request'].message_type = labust__pb2._NANOMODEMREQUEST
DESCRIPTOR.message_types_by_name['CommandRequest'] = _COMMANDREQUEST
DESCRIPTOR.message_types_by_name['AcousticResponse'] = _ACOUSTICRESPONSE
DESCRIPTOR.message_types_by_name['AcousticRequest'] = _ACOUSTICREQUEST
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

CommandRequest = _reflection.GeneratedProtocolMessageType('CommandRequest', (_message.Message,), dict(
  DESCRIPTOR = _COMMANDREQUEST,
  __module__ = 'acoustic_transmission_pb2'
  # @@protoc_insertion_point(class_scope:acoustictransmission.CommandRequest)
  ))
_sym_db.RegisterMessage(CommandRequest)

AcousticResponse = _reflection.GeneratedProtocolMessageType('AcousticResponse', (_message.Message,), dict(
  DESCRIPTOR = _ACOUSTICRESPONSE,
  __module__ = 'acoustic_transmission_pb2'
  # @@protoc_insertion_point(class_scope:acoustictransmission.AcousticResponse)
  ))
_sym_db.RegisterMessage(AcousticResponse)

AcousticRequest = _reflection.GeneratedProtocolMessageType('AcousticRequest', (_message.Message,), dict(
  DESCRIPTOR = _ACOUSTICREQUEST,
  __module__ = 'acoustic_transmission_pb2'
  # @@protoc_insertion_point(class_scope:acoustictransmission.AcousticRequest)
  ))
_sym_db.RegisterMessage(AcousticRequest)



_ACOUSTICTRANSMISSION = _descriptor.ServiceDescriptor(
  name='AcousticTransmission',
  full_name='acoustictransmission.AcousticTransmission',
  file=DESCRIPTOR,
  index=0,
  serialized_options=None,
  serialized_start=341,
  serialized_end=549,
  methods=[
  _descriptor.MethodDescriptor(
    name='StreamAcousticRequests',
    full_name='acoustictransmission.AcousticTransmission.StreamAcousticRequests',
    index=0,
    containing_service=None,
    input_type=_COMMANDREQUEST,
    output_type=_ACOUSTICREQUEST,
    serialized_options=None,
  ),
  _descriptor.MethodDescriptor(
    name='ReturnAcousticPayload',
    full_name='acoustictransmission.AcousticTransmission.ReturnAcousticPayload',
    index=1,
    containing_service=None,
    input_type=_ACOUSTICRESPONSE,
    output_type=std__pb2._EMPTY,
    serialized_options=None,
  ),
])
_sym_db.RegisterServiceDescriptor(_ACOUSTICTRANSMISSION)

DESCRIPTOR.services_by_name['AcousticTransmission'] = _ACOUSTICTRANSMISSION

# @@protoc_insertion_point(module_scope)
