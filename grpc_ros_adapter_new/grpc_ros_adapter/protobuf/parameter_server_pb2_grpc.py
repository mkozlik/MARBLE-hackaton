# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
import grpc

import parameter_server_pb2 as parameter__server__pb2
import std_pb2 as std__pb2


class ParameterServerStub(object):
  # missing associated documentation comment in .proto file
  pass

  def __init__(self, channel):
    """Constructor.

    Args:
      channel: A grpc.Channel.
    """
    self.GetParameter = channel.unary_unary(
        '/parameterserver.ParameterServer/GetParameter',
        request_serializer=parameter__server__pb2.GetParamRequest.SerializeToString,
        response_deserializer=parameter__server__pb2.ParamValue.FromString,
        )
    self.SetParameter = channel.unary_unary(
        '/parameterserver.ParameterServer/SetParameter',
        request_serializer=parameter__server__pb2.SetParamRequest.SerializeToString,
        response_deserializer=std__pb2.Empty.FromString,
        )


class ParameterServerServicer(object):
  # missing associated documentation comment in .proto file
  pass

  def GetParameter(self, request, context):
    # missing associated documentation comment in .proto file
    pass
    context.set_code(grpc.StatusCode.UNIMPLEMENTED)
    context.set_details('Method not implemented!')
    raise NotImplementedError('Method not implemented!')

  def SetParameter(self, request, context):
    # missing associated documentation comment in .proto file
    pass
    context.set_code(grpc.StatusCode.UNIMPLEMENTED)
    context.set_details('Method not implemented!')
    raise NotImplementedError('Method not implemented!')


def add_ParameterServerServicer_to_server(servicer, server):
  rpc_method_handlers = {
      'GetParameter': grpc.unary_unary_rpc_method_handler(
          servicer.GetParameter,
          request_deserializer=parameter__server__pb2.GetParamRequest.FromString,
          response_serializer=parameter__server__pb2.ParamValue.SerializeToString,
      ),
      'SetParameter': grpc.unary_unary_rpc_method_handler(
          servicer.SetParameter,
          request_deserializer=parameter__server__pb2.SetParamRequest.FromString,
          response_serializer=std__pb2.Empty.SerializeToString,
      ),
  }
  generic_handler = grpc.method_handlers_generic_handler(
      'parameterserver.ParameterServer', rpc_method_handlers)
  server.add_generic_rpc_handlers((generic_handler,))
