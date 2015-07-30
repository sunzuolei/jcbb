function showResult(result,step)
% Visulize the result of data association
% 

t = step;
figure(2); hold on;
subplot(2, 2, 1); hold on;
bar(result.tp./result.total); axis([1 t 0 1]);
title('True positives');

subplot(2, 2, 2); hold on;
bar(result.tn./result.total); axis([1 t 0 1]);
title('True negatives');

subplot(2, 2, 3); hold on;
bar(result.fp./result.total); axis([1 t 0 1]);
title('False positives');

subplot(2, 2, 4); hold on;
bar(result.fn./result.total); axis([1 t 0 1]);
title('False negatives');


figure(3);hold on;
subplot(2, 2, 1); hold on;
bar(result.tp./(result.tp + result.fp)); axis([1 t 0 1]);
title('Precision');

subplot(2, 2, 2); hold on;
bar(result.tp./(result.tp + result.fn)); axis([1 t 0 1]);
title('Recall');

subplot(2, 2, 3); hold on;
bar((2.*result.tp)./((2.*result.tp + result.fn + result.fp))); axis([1 t 0 1]);
title('F1Score');

subplot(2, 2, 4); hold on;
bar((result.tp + result.tn)./((result.tp + result.fn + result.fp + result.tn))); axis([1 t 0 1]);
title('Accuracy');
end

