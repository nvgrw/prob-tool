open Arg

let () =
  let filename = ref None in
  parse [] (fun anonymous -> filename := Some anonymous) (Printf.sprintf "%s" Sys.argv.(0));
  let in_file = match !filename with
    | None ->
      prerr_endline "no input file provided! aborting...";
      exit 1
    | Some f -> f
  in
  let con = Llvm.global_context () in
  let buf = Llvm.MemoryBuffer.of_file in_file in
  let mdl = Llvm_bitreader.parse_bitcode con buf in
  Llvm.MemoryBuffer.dispose buf;
  Analysis.analyze mdl;
  print_endline @@ Printf.sprintf "Analyzed %s." in_file