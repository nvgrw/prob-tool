
let inst i =
  print_endline @@ Llvm.string_of_llvalue i

let block b =
  Printf.printf "%%%s:\n" (Llvm.value_name @@ Llvm.value_of_block b);
  Llvm.iter_instrs inst b

let func f =
  if not (Llvm.is_declaration f) then (
    Printf.printf "(%s)\n" (Llvm.value_name f);
    Llvm.iter_blocks block f)
  else ()

let analyze mdl =
  let pass_manager = Llvm.PassManager.create () in
  (* passes *)
  Llvm_scalar_opts.add_gvn pass_manager;
  Llvm_scalar_opts.add_memory_to_register_demotion pass_manager;
  (* end passes *)
  ignore @@ Llvm.PassManager.run_module mdl pass_manager;
  Llvm.PassManager.dispose pass_manager;
  Llvm.iter_functions func mdl;
  ()